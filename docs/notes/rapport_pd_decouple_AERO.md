# Commande PD découplée de l'AERO (2 DDL) — théorie, conception et simulation

*Mini-projet Quanser AERO — Polytech Nancy — 14/10/2026*
*Script : `decoupled_pd_AERO.m` — figures : `fig1_suivi_rampes.png` … `fig4_couplage_scenarios.png`*

---

## 0. Résumé

- **Le couplage de l'AERO est entièrement statique** dans le modèle Quanser identifié : les deux « corps » (tangage, lacet) ne sont pas couplés, seuls les gains de couple des moteurs le sont. Un **découpleur statique** `W = Bt⁻¹` supprime donc exactement G12 et G21 dans le modèle, sans filtre ni dynamique supplémentaire.
- Sur chaque axe découplé, un **PD (forme PV, dérivée sur la mesure)** est réglé avec les formules Quanser (`quanser_aero_pd_design_Ksp.m`) écrites en couple, plus une **anticipation du ressort** `Ksp·r_p` : erreur statique nulle en tangage dans le modèle.
- Le contrôleur complet s'écrit `V = Keq·(xd − x) + Nff·r` : **même schéma Simulink que le LQR** (on remplace K par Keq et on ajoute le gain de pré-commande Nff).
- Ajout d'une **saturation « priorité tangage »** : quand une tension atteint ±20 V, on réduit seulement le couple de lacet, ce qui conserve le découplage. Sans elle, un échelon de lacet fait chuter le tangage de 24° ; avec elle, de 0,5°.
- **Simulation (modèle réaliste : frottement sec, retards moteurs, quantification des codeurs, filtre de vitesse QUARC), consignes en rampe** — PD découplé + priorité vs LQR « Current » :
  - couplage lacet → tangage : **0,23° vs 1,85°** ; couplage tangage → lacet : **0,49° vs 2,24°** ;
  - erreur statique tangage : **0,06° vs 1,92°** ;
  - dépassement lacet : **0,7 % vs 7,7 %** ; RMSE équivalente (3,4/4,5° vs 3,2/4,2°) ;
  - activité des tensions : **48 V/s vs 68 V/s**.
- **Limites** : le découplage n'est exact que si `Kpy` et `Kyp` sont justes (les paramètres les moins sûrs, G12 à ~60 % de fit) et si la poussée est linéaire en V. Avec une poussée en V|V| simulée, le découplage se dégrade (couplage tangage ≈ celui du LQR). On fournit deux facteurs de réglage `cKpy`, `cKyp` et une règle de signe pour les corriger sur l'AERO.
- **Ce qui reste à faire** : essai sur l'AERO, calibration de `cKpy`/`cKyp`, puis action intégrale (PID découplé avec anti-windup) pour l'erreur résiduelle du lacet due au frottement sec.

> **Validation** : MATLAB n'étant pas disponible de mon côté, le script a été validé par un **portage Python ligne à ligne** (mêmes équations, mêmes indicateurs, mêmes paramètres lus dans les `.mat`). Les nombres de ce rapport sont ceux que le script MATLAB doit afficher, aux arrondis numériques près. À confirmer à la première exécution.

---

## 1. D'où vient le couplage ?

### 1.1 Le modèle (structure Quanser conservée)

$$J_p\ddot\theta + D_p\dot\theta + K_{sp}\theta = \tau_p,\qquad J_y\ddot\psi + D_y\dot\psi = \tau_y$$

$$\begin{bmatrix}\tau_p\\ \tau_y\end{bmatrix} = \underbrace{\begin{bmatrix}K_{pp} & K_{py}\\ K_{yp} & K_{yy}\end{bmatrix}}_{B_t}\begin{bmatrix}V_p\\ V_y\end{bmatrix}$$

Avec le jeu **linéaire identifié (CONTSID)**, utilisé pour la conception :

| | Valeur | | Valeur |
|---|---|---|---|
| Kpp | 7,379·10⁻⁴ N·m/V | Kpy | 6,126·10⁻⁴ N·m/V |
| Kyp | −6,450·10⁻⁴ N·m/V | Kyy | 3,546·10⁻⁴ N·m/V |
| Ksp | 0,03069 N·m/rad | Dp, Dy | 0,00845 / 0,01154 N·m·s/rad |
| Jp, Jy | 0,02189 / 0,02199 kg·m² | | |

### 1.2 Le point clé : G(s) = diag(Pp, Py) · Bt

En notant $P_p(s) = \dfrac{1}{J_p s^2 + D_p s + K_{sp}}$ et $P_y(s) = \dfrac{1}{s(J_y s + D_y)}$ :

$$G(s) = \begin{bmatrix}G_{11} & G_{12}\\ G_{21} & G_{22}\end{bmatrix} = \begin{bmatrix}K_{pp}P_p & K_{py}P_p\\ K_{yp}P_y & K_{yy}P_y\end{bmatrix} = \begin{bmatrix}P_p & 0\\ 0 & P_y\end{bmatrix} B_t$$

Les termes croisés ont **la même dynamique** que les termes directs : seule leur amplitude diffère. Physiquement, chaque hélice crée un couple sur les deux axes (poussée + couple de réaction), mais il n'y a qu'un seul « corps » par axe. Dans la matrice d'état, A est diagonale par blocs (pas de terme croisé) et tout le couplage est dans B.

### 1.3 Mesure de l'interaction : la RGA

La matrice des gains relatifs (RGA) vaut $\Lambda = G \circ (G^{-1})^T$. Comme $G = \mathrm{diag}(P)\,B_t$ et que la RGA est insensible à une mise à l'échelle diagonale :

$$\Lambda(s) = B_t \circ (B_t^{-1})^T = \begin{bmatrix}0{,}40 & 0{,}60\\ 0{,}60 & 0{,}40\end{bmatrix}\quad\text{à toutes les fréquences.}$$

- λ₁₁ = 0,40 < 0,5 : **forte interaction**. Un PD indépendant par axe voit son gain effectif multiplié par 1/λ₁₁ ≈ 2,5 quand l'autre boucle est fermée : les réglages calculés axe par axe ne sont plus valables.
- λ₁₁ > 0 : pas d'inversion de signe, l'appariement Vp→θ, Vy→ψ reste possible, mais il n'est pas « naturellement » dominant.
- cond(Bt) = 1,60 : la matrice est bien conditionnée, **l'inverser est sans danger** (pas de gains énormes créés par l'inversion).

Avec les paramètres Quanser d'origine, λ₁₁ ≈ 0,30 : le constat est le même, en plus marqué.

---

## 2. Le découpleur

### 2.1 Inversion statique

On commande des **couples** et on calcule les tensions par :

$$V = W\,\tau_c,\qquad W = B_t^{-1} = \begin{bmatrix}540 & -933\\ 982 & 1123\end{bmatrix}\ \text{V/(N·m)}$$

Alors $G(s)\,W = \mathrm{diag}(P_p, P_y)$ : **G12 et G21 disparaissent** (dans le modèle).

### 2.2 Lien avec le découplage « classique » du cours

La méthode simplifiée $D_{12} = -G_{12}/G_{11}$, $D_{21} = -G_{21}/G_{22}$ donne ici des **constantes** :

$$D_{12} = -K_{py}/K_{pp} = -0{,}83,\qquad D_{21} = -K_{yp}/K_{yy} = +1{,}82$$

et $G\,D = \mathrm{diag}\big(P_p\,\det B_t/K_{yy},\ P_y\,\det B_t/K_{pp}\big)$. C'est la même chose que $B_t^{-1}$ à une mise à l'échelle près ; on retrouve d'ailleurs le facteur $1/\lambda_{11} = 2{,}5$ : les deux moteurs **coopèrent**, le gain effectif par axe est 2,5 fois plus grand qu'avec un seul moteur.

### 2.3 Interprétation physique des colonnes de W

- **Couple de tangage pur** : $V_p = 540\,\tau$, $V_y = 982\,\tau$. Le moteur de lacet aide à lever (Kpy > 0) et annule le couple de réaction du moteur de tangage sur le lacet (Kyp < 0).
- **Couple de lacet pur** : $V_p = -933\,\tau$, $V_y = 1123\,\tau$. Le moteur de tangage tourne à l'envers : son couple de réaction (Kyp·Vp > 0) aide le lacet, et sa poussée annule celle du moteur de lacet sur le tangage.
- Tenir le tangage à 20° demande $\tau = K_{sp}\cdot 0{,}349 = 0{,}0107$ N·m, soit **Vp = 5,8 V et Vy = 10,5 V**. Le lacet n'est pas « au repos » : il compense activement.

---

## 3. Le PD sur chaque axe découplé

### 3.1 Loi de commande (forme PV)

$$\tau_p = k_{p,p}(r_p - \theta) - k_{d,p}\dot\theta + K_{sp}r_p,\qquad \tau_y = k_{p,y}(r_y - \psi) - k_{d,y}\dot\psi$$

- La dérivée agit sur la **mesure** (pas sur l'erreur) : pas de « coup de pied » à un échelon, et pas de zéro en boucle fermée, donc la réponse suit exactement la formule du 2ᵉ ordre.
- **Anticipation du ressort** `Ksp·r_p` : c'est le couple de maintien. En boucle fermée, $\theta_\infty = \frac{K_{sp}+k_{p,p}}{K_{sp}+k_{p,p}}\,r_p = r_p$. Sans elle, l'erreur statique serait $K_{sp}/(J_p\omega_n^2) = 26\,\%$ de la consigne. C'est l'équivalent découplé du gain de pré-commande `Nu` déjà étudié pour le LQR.
- Le lacet n'a pas de ressort (intégrateur dans Py) : pas d'erreur statique dans le modèle linéaire.

### 3.2 Placement des pôles (formules Quanser en couple)

Boucle fermée : $J s^2 + (D + k_d)s + (K_s + k_p) = J(s^2 + 2\zeta\omega_n s + \omega_n^2)$, d'où

$$k_{p,p} = J_p\omega_n^2 - K_{sp},\quad k_{d,p} = 2\zeta\omega_n J_p - D_p,\qquad k_{p,y} = J_y\omega_n^2,\quad k_{d,y} = 2\zeta\omega_n J_y - D_y$$

Ce sont les formules de `quanser_aero_pd_design_Ksp.m` multipliées par Kpp (resp. Kyy) : on règle en N·m au lieu de V, et le découpleur fait la conversion.

Spécifications Quanser : D = 7,5 %, tₚ = 1,75 s → **ζ = 0,636, ωₙ = 2,33 rad/s** (pôles −1,48 ± 1,80j sur chaque axe).

| Gain | Tangage | Lacet |
|---|---|---|
| kp (N·m/rad) | 0,0878 | 0,1191 |
| kd (N·m·s/rad) | 0,0563 | 0,0536 |

### 3.3 Forme matricielle — même schéma que le LQR

$$V = K_{eq}(x_d - x) + N_{ff}\,r,\qquad x_d = [r_p;\ r_y;\ 0;\ 0]$$

$$K_{eq} = W\begin{bmatrix}k_{p,p} & 0 & k_{d,p} & 0\\ 0 & k_{p,y} & 0 & k_{d,y}\end{bmatrix} = \begin{bmatrix}47{,}4 & -111{,}0 & 30{,}4 & -50{,}0\\ 86{,}2 & 133{,}7 & 55{,}3 & 60{,}2\end{bmatrix},\qquad N_{ff} = W\begin{bmatrix}K_{sp} & 0\\ 0 & 0\end{bmatrix} = \begin{bmatrix}16{,}6 & 0\\ 30{,}1 & 0\end{bmatrix}$$

Pour comparaison, le LQR « Current » : $K = \begin{bmatrix}98{,}2 & -103{,}1 & 32{,}3 & -29{,}1\\ 156{,}4 & 66{,}2 & 45{,}5 & 17{,}1\end{bmatrix}$.

**Même structure de signes** que le LQR (rassurant avant l'essai) : le LQR faisait déjà un découplage partiel, mais calculé sur le modèle Quanser (Kpy 3,4 fois trop grand) et sans contrainte de découplage.

Pôles en boucle fermée sur le modèle identifié : PD découplé −1,48 ± 1,80j (double) ; LQR −1,41 ± 2,68j et **−0,79 ± 1,84j** (plus lent, ζ ≈ 0,40).

### 3.4 Pourquoi le PD « indépendant » de Quanser ne suffit pas

Le même script Quanser appliqué au modèle identifié **sans découpleur** donne $K = \mathrm{diag}$ avec kp,y = 336 V/rad et kd,y = 151 V·s/rad (Kyy identifié 6 fois plus petit que celui de Quanser). En boucle fermée, à cause de λ₁₁ = 0,4, les pôles réels deviennent −1,22 ± 3,93j (ζ ≈ 0,30 au lieu de 0,64) : plus oscillant que prévu, très bruité (gain de vitesse ×2,5), et le couplage reste entier.

### 3.5 Saturation qui conserve le découplage (« priorité tangage »)

Si une tension est saturée, le couple réel n'est plus `τ_c` : le découplage est perdu au pire moment (grands mouvements). On garde **exactement le couple de tangage** et on réduit seulement le couple de lacet. Pour chaque moteur i :

$$|W_{i1}\tau_p + W_{i2}\tau_y| \le 20\ \text{V}\ \Rightarrow\ \tau_y \in \left[\frac{-20 - W_{i1}\tau_p}{W_{i2}},\ \frac{20 - W_{i1}\tau_p}{W_{i2}}\right]\ \text{(bornes inversées si } W_{i2}<0)$$

On prend l'intersection des deux intervalles et on borne τ_y. Pourquoi le tangage en priorité : c'est l'axe qui peut taper la base, il porte le ressort (couple de maintien indispensable), et c'est le couplage lacet → tangage que le professeur a signalé comme fort. Le lacet ralentit simplement un peu pendant la saturation.

### 3.6 Marges de stabilité (par axe, filtre State X et retard moteur inclus)

| Axe | Coupure | Marge de phase | Marge de gain | Marge de retard |
|---|---|---|---|---|
| Tangage (retard 50 ms) | 3,3 rad/s | 58° | 17 dB | 0,30 s |
| Lacet (retard 100 ms) | 3,1 rad/s | 42° | 12 dB | 0,24 s |
| Lacet, gain réel ×1,4 (jeu « frottement ») | 4,1 rad/s | 35° | 9 dB | 0,15 s |

Marges confortables. Le lacet est l'axe le plus sensible (retard plus grand, gain incertain). Avec tₚ = 1,5 s, la marge de phase du lacet tombe à 38° (30° si le gain réel est ×1,4).

### 3.7 Bandes de blocage dues au frottement sec

Un PD seul ne peut pas vaincre un couple de Coulomb plus grand que $k_p\cdot e$ :

- tangage : $F_{c,p}/(K_{sp}+k_{p,p}) = 0{,}00057/0{,}1185$ → **±0,3°** ;
- lacet : $F_{c,y}/k_{p,y} = 0{,}00415/0{,}119$ → jusqu'à **±2°** (±1,4° si le gain réel est ×1,4).

C'est ce qui reste à corriger par une action intégrale.

---

## 4. Simulation

### 4.1 Conditions

- Contrôleur discret Ts = 2 ms, saturation ±20 V.
- Vitesses estimées comme dans le bloc **State X** du modèle Simulink : filtre QUARC « Second-Order Low-Pass Filter », ωₙ = 50 rad/s, ζ = 0,8 (relevé dans `q_aero_2dof_lqr_control.slx`).
- **Procédés** :
  - *linéaire* : modèle identifié (celui de la conception) ;
  - *réaliste* : jeu « frottement » (Ksp, Dp, Kpp, Fc_p ; Dy, Kyy, Fc_y avec frottement statique de Karnopp), gains croisés CONTSID, retards 50 ms (tangage) et 100 ms (lacet), quantification des codeurs (0,176° / 0,088°) ;
  - *réaliste + V²* : poussée en V|V|/8 (égale au modèle linéaire à 8 V).
- **Consignes décalées** : un seul axe bouge à la fois, sinon le couplage ne peut pas être mesuré (défaut connu de la métrique actuelle de `tune_lqr_model_refined.m`). Tangage 0 → +20 → −20 → 0° (t = 1, 13, 26 s), lacet 0 → +40 → −40 → 0° (t = 6, 18, 31 s), Rate Limiter 20°/s (ou échelons).
- **Indicateurs** : e = erreur statique (dernière seconde avant le mouvement suivant) ; D = dépassement ; RMSE ; cpl = plus grand déplacement d'un axe pendant un mouvement de l'autre (fenêtre = rampe + 3 s) ; sat = % d'échantillons saturés ; activité = Σ|ΔV|/T des deux moteurs.

Contrôleurs : PD indépendant (scripts Quanser, sans découpleur ni anticipation), PD découplé, PD découplé + priorité tangage, PD découplé plus rapide (tₚ = 1,5 s), LQR « Current » (K utilisé sur l'AERO).

### 4.2 Résultats — procédé réaliste, rampes (cas le plus proche de l'AERO)

| Contrôleur | e_θ (°) | e_ψ (°) | D_θ (%) | D_ψ (%) | RMSE θ/ψ (°) | cpl θ (°) | cpl ψ (°) | sat (%) | activité (V/s) |
|---|---|---|---|---|---|---|---|---|---|
| PD indépendant (Quanser) | 2,40 | 0,40 | 8,2 | 1,8 | 4,22 / 5,21 | 4,80 | 1,86 | 8,1 | 99,7 |
| PD découplé | 0,10 | 0,46 | 4,6 | 0,7 | 3,38 / 4,50 | 0,51 | 0,53 | 1,4 | 49,8 |
| **PD découplé + priorité** | **0,06** | **0,44** | **4,6** | **0,7** | **3,38 / 4,53** | **0,23** | **0,49** | **1,5** | **48,5** |
| PD découplé, tₚ = 1,5 s | 0,06 | 0,33 | 4,0 | 0,4 | 2,92 / 4,01 | 0,23 | 0,29 | 2,0 | 56,2 |
| LQR « Current » | 1,92 | 0,79 | 5,4 | 7,7 | 3,18 / 4,20 | 1,85 | 2,24 | 1,5 | 68,0 |

![Suivi des rampes](fig1_suivi_rampes.png)

![Erreurs et couplage](fig2_erreurs_couplage.png)

Lecture :

- **Couplage divisé par 8 (lacet → tangage) et par 4,5 (tangage → lacet)** par rapport au LQR, par 20 et 4 par rapport au PD indépendant. Sur la figure 2, les « bosses » de l'erreur de tangage aux pointillés (début d'un mouvement de lacet) disparaissent presque avec le PD découplé.
- **Erreur statique de tangage quasi nulle** grâce à l'anticipation du ressort (le LQR garde ~2°, comme mesuré sur l'AERO : 2,7–3,0°).
- **RMSE équivalente au LQR** : elle est dominée par le retard de poursuite pendant les rampes (~2ζ/ωₙ × 20°/s ≈ 11° pour le PD, du même ordre pour le LQR qui impose aussi une vitesse de consigne nulle). Le PD n'est pas plus rapide, il est mieux découplé et plus précis.
- **Activité des tensions plus faible que le LQR** (48 vs 68 V/s) malgré un gain de vitesse maximal plus grand (60 vs 46 V·s/rad) : les gains de vitesse du PD sont répartis différemment entre les moteurs. En valeur absolue, la simulation surestime l'activité (LQR sur échelons : 72 V/s simulés contre 43 V/s mesurés), seule la comparaison relative compte.
- **Lacet** : dépassement 0,7 % au lieu de 7,7 %, mais erreur statique de ~0,4° (frottement sec, § 3.7).

### 4.3 Robustesse : les trois autres cas

| Contrôleur | Linéaire, rampes : cpl θ / ψ | Réaliste + V², rampes : cpl θ / ψ | Réaliste, **échelons** : cpl θ / ψ | Réaliste, échelons : sat (%) |
|---|---|---|---|---|
| PD indépendant | 3,82 / 3,15 | 8,76 / 8,78 | 6,47 / 2,74 | 19,5 |
| PD découplé (saturation simple) | 0,90 / 0,01 | 1,65 / 1,39 | **24,18** / 2,16 | 14,1 |
| **PD découplé + priorité** | **0,07 / 0,01** | **1,65 / 1,39** | **0,48 / 1,38** | 9,4 |
| LQR « Current » | 1,89 / 0,60 | 1,60 / 2,76 | 21,30 / 0,90 | 23,3 |

![Couplage selon les cas](fig4_couplage_scenarios.png)

- **Linéaire** : le couplage résiduel (0,9°) du PD découplé simple vient uniquement de la saturation ; la priorité tangage le ramène à 0,07°.
- **Échelons** : la saturation est fréquente (9–23 %). Sans priorité, le découplage s'effondre (24° de chute du tangage pour un échelon de lacet de 80°, comme le LQR : 21°, à comparer aux 12,6–16,2° mesurés). Avec la priorité tangage : **0,5°**. C'est la contribution la plus visible de ce travail (figure 3).
- **Poussée en V²** : le gain par volt dépend du point de fonctionnement (pente locale |V|/4 : ×1,45 à 5,8 V, ×2,6 à 10,5 V). Le découpleur, calculé pour un point moyen à 8 V, n'est plus exact : couplage tangage ≈ celui du LQR, couplage lacet encore 2 fois plus faible. Le PD indépendant, lui, sature 60 % du temps.

![Échelons et saturation](fig3_echelons_saturation.png)

### 4.4 Sensibilité aux gains croisés (procédé réaliste, rampes)

On garde le découpleur et on multiplie Kpy, Kyp **du procédé** :

| Kpy × | Kyp × | PD découplé + priorité : cpl θ / ψ (°) | LQR : cpl θ / ψ (°) |
|---|---|---|---|
| 1 | 1 | 0,23 / 0,49 | 1,85 / 2,24 |
| 0,5 | 1 | 1,37 / 1,30 | 5,43 / 1,13 |
| 1,5 | 1 | 0,95 / 0,04 | 0,55 / 1,84 |
| 1 | 0,5 | 0,24 / 3,07 | 2,75 / 4,69 |
| 1 | 1,5 | 0,24 / 0,00 | 1,52 / 1,12 |
| 0,5 | 0,5 | 1,63 / 3,95 | 6,90 / 5,18 |
| 1,5 | 1,5 | 0,91 / 0,00 | 0,43 / 1,03 |

- Une erreur sur **Kpy** se voit sur le **tangage**, une erreur sur **Kyp** sur le **lacet** (le couplage résiduel est proportionnel à l'erreur de gain).
- Même avec ±50 % d'erreur, le PD découplé reste en général meilleur que le LQR. Le LQR est meilleur quand Kpy est plus grand que l'identifié, ce qui est logique : il a été calculé sur le modèle Quanser (Kpy = 0,0021, 3,4 fois plus grand).
- Le frottement sec du lacet masque une partie de l'erreur sur Kyp (le lacet reste collé tant que le couple parasite est inférieur à Fc).

### 4.5 Autres essais (non retenus)

- **Anticipation de la vitesse de consigne** (k_d·ṙ + D·ṙ) : divise la RMSE par ~2 en suivant la rampe, mais donne 15–22 % de dépassement en fin de rampe (il manque le terme d'accélération J·r̈, impulsionnel aux coins de la rampe) et sature davantage. À reprendre avec une consigne plus lisse (pré-filtre du 2ᵉ ordre).
- **Inversion de la poussée** (V = sign(u)·√(8|u|)) : si l'hypothèse V² est vraie, elle restitue exactement les performances du cas linéaire (couplage 0,23/0,49°) ; si elle est fausse, elle dégrade fortement le lacet (RMSE ×2,5). Non incluse dans le script : à tester seulement après avoir confirmé l'effet V² en boucle fermée.

---

## 5. Mise en œuvre dans Simulink

### Option A — rapide (PD découplé, saturation simple)

Dans le modèle LQR existant (Aero_RW + State X) :

1. Remplacer le gain **K** par **Keq** (affiché par le script et sauvé dans `results_decoupled_pd.mat`, variable `KeqAll{2}`).
2. Ajouter un gain **Nff** (`NffAll{2}`) sur le vecteur de consigne [r_p ; r_y] **en radians**, et l'additionner à la sortie de Keq (si le bloc de pré-commande `Nu` existe déjà, il suffit de remplacer sa valeur).
3. Garder la saturation ±20 V avant HIL Write.

### Option B — avec priorité tangage (recommandée)

1. Gain **Kt** (`KtAll{3}`) sur (x_d − x) → couples ; gain **Ft** (`FtAll{3}`) sur la consigne ; somme → τ (2×1, en N·m).
2. Bloc **MATLAB Function** (entrées : `tau`, et trois Constant : `W`, `satV` = 20, `prio` = 1) :

```matlab
function V = fcn(tau, W, satV, prio)
% Couples -> tensions avec saturation "priorité tangage" (voir rapport, § 3.5)
if prio
    lo = -inf;  hi = inf;
    for i = 1:2
        if W(i,2) == 0, continue; end
        a = W(i,1)*tau(1);
        l = (-satV - a) / W(i,2);
        h = ( satV - a) / W(i,2);
        if W(i,2) < 0
            tmp = l;  l = h;  h = tmp;
        end
        lo = max(lo, l);
        hi = min(hi, h);
    end
    if lo <= hi
        tau(2) = min(max(tau(2), lo), hi);
    end
end
V = W*tau;
V = min(max(V, -satV), satV);
```

3. Garder la saturation ±20 V en sécurité après le bloc.

### Vérifications avant de lancer

- Unités : x et consignes en **radians** dans la boucle (erreur déjà rencontrée avec le bloc State X).
- Signes de Keq identiques à ceux du LQR qui fonctionne : [+ − + − ; + + + +].
- Premier essai à petite amplitude (tangage ±10°, lacet ±20°, rampes), la main près de l'arrêt.

---

## 6. Protocole d'essai sur l'AERO et calibration du découpleur

1. **Essai 1** : option A, consignes décalées du script (ou celles du Simulink si un seul axe bouge à la fois), rampes 20°/s. Enregistrer le log à 9 lignes.
2. Mettre le nom du fichier dans `measFile` et `iMeas = 2` : le script trace la mesure en noir, calcule les mêmes indicateurs et le fit simulation/mesure.
3. **Calibration** des gains croisés avec `cKpy`, `cKyp` (le découpleur utilise cKpy·Kpy et cKyp·Kyp). Le couple parasite vaut $(K_{réel} - K_{découpleur})\times W$, d'où les règles :

| Ce qu'on observe | Cause | Correction |
|---|---|---|
| Le **tangage monte** au début d'un mouvement de lacet **positif** | Kpy réel > Kpy du découpleur | **augmenter** cKpy |
| Le tangage descend au début d'un mouvement de lacet positif | Kpy réel < Kpy du découpleur | diminuer cKpy |
| Le **lacet tourne en positif** quand le tangage monte (ou reste haut) | Kyp réel moins négatif que celui du découpleur | **diminuer** cKyp |
| Le lacet tourne en négatif quand le tangage monte | Kyp réel plus négatif | augmenter cKyp |

   Procéder par pas de ±0,25 et garder la valeur qui minimise cpl_θ (resp. cpl_ψ). Pour le lacet, utiliser de grands mouvements de tangage : en dessous de ~6 V équivalents, le frottement sec masque le couple parasite.
4. **Essai 2** : option B (priorité tangage), mêmes consignes, puis en échelons pour vérifier la tenue du tangage pendant un échelon de lacet.
5. Répéter chaque essai au moins deux fois (dispersion d'un essai à l'autre encore inconnue).

---

## 7. Suite proposée

1. **PID découplé avec anti-windup** : sur chaque axe découplé, ajouter un intégrateur (sur le lacet surtout, pour l'erreur de ~0,4–2° due au frottement sec). Avec la priorité tangage, l'anti-windup du lacet est simple : arrêter l'intégration quand τ_y a été réduit.
2. Si l'effet V² se confirme en boucle fermée : inversion de la poussée ou découpleur séquencé selon le point de fonctionnement.
3. Consigne plus lisse (pré-filtre du 2ᵉ ordre) + anticipation vitesse/accélération, pour réduire la RMSE de poursuite.
4. Réutiliser `computeMetrics.m` sur les essais mesurés pour comparer avec les campagnes LQR (le script contient ses propres indicateurs, nécessaires pour mesurer le couplage avec des consignes décalées).

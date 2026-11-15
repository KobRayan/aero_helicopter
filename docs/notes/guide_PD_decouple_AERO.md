# Guide complet — Correcteur PD découplé sur le Quanser AERO

*Mini-projet AERO, Polytech Nancy — préparation de la soutenance du 16/10/2026*

Ce guide reprend **tout** le travail sur le PD découplé : l'idée, les formules, la mise en œuvre, les indicateurs, les essais, le réglage `cKyp`, les limites physiques et la comparaison avec le LQR. Il se termine par les formules LaTeX prêtes à coller, et par les questions probables du jury.

**Fichiers associés**
- `figures_pd_presentation_AERO.m` : les 4 figures propres (`fig_pd_1…4.png`).
- `schema_pd_decouple.png` : schéma de principe.
- `decoupled_pd_AERO.m` : conception et simulation du PD.
- `setup_decoupled_pd_simulink.m` : variables du modèle Simulink.
- `plot_decoupled_pd_tests_AERO.m` : dépouillement détaillé des essais.

---

## 0. À retenir en 30 secondes

1. **Tout le couplage de l'AERO est dans les moteurs, pas dans la mécanique.** Chaque hélice crée un couple sur les deux axes. Le modèle s'écrit $G(s) = \mathrm{diag}(P_p, P_y)\,B_t$.
2. **Le découpleur inverse ce mélange** : $V = B_t^{-1}\tau$. Chaque PD commande alors « son » couple et ne voit que « son » axe.
3. **Un PD par axe**, réglé avec les formules de Quanser ($D$ = 7,5 %, $t_p$ = 1,75 s), plus un **feedforward du ressort** (bloc `SpringFF` dans Simulink, aussi appelé anticipation ou pré-commande) $K_{sp}\,r_p$ : erreur statique nulle dans le modèle, et **0,9° mesuré au lieu de ~6° sans lui** (§ 3.4).
4. **Sur l'AERO**, l'erreur statique de tangage est **3 fois plus faible qu'avec le LQR** (0,9° contre 2,7°). En revanche, **le couplage réel n'est pas plus faible** qu'avec le LQR, alors que la simulation le prédisait 3 à 4 fois plus faible.
5. **Pourquoi ?** Le découpleur est **statique** et repose sur $K_{py}$ et $K_{yp}$, les paramètres les moins sûrs du modèle. Les moteurs ne donnent que 63–84 % du couple prévu, avec la poussée en $V^2$. Les « coups » de couple des hélices quand elles accélèrent ne peuvent pas être compensés statiquement. Le réglage `cKyp` = 0,75 a réduit la dérive du lacet de 40 %.

---

## 1. Le problème : pourquoi les deux axes se gênent

### 1.1 Ce qu'on voit sur la maquette

En boucle ouverte, une tension sur le moteur de lacet fait bouger le tangage **autant** que le moteur de tangage lui-même. Une tension sur le moteur de tangage fait tourner le lacet **à l'envers**. C'est un système **MIMO couplé** : 2 entrées ($V_p$, $V_y$) et 2 sorties ($\theta$, $\psi$), et chaque entrée agit sur les deux sorties.

**L'origine physique : les couples des hélices.** Une hélice qui tourne pousse (c'est la poussée) mais, par réaction, fait aussi tourner le corps dans l'autre sens. Comme les deux rotors sont de même taille et à la même distance du pivot, chacun crée un couple sur les deux axes.

### 1.2 Le modèle (structure Quanser, paramètres identifiés avec CONTSID)

$$J_p\ddot\theta + D_p\dot\theta + K_{sp}\theta = \underbrace{K_{pp}V_p + K_{py}V_y}_{\tau_p}
\qquad
J_y\ddot\psi + D_y\dot\psi = \underbrace{K_{yp}V_p + K_{yy}V_y}_{\tau_y}$$

À gauche, les **corps** (inertie, frottement visqueux, ressort du tangage) ; ils sont **indépendants**. À droite, les **couples moteurs** ; c'est **là que se trouve le mélange**. En écriture matricielle :

$$\begin{bmatrix}\tau_p\\ \tau_y\end{bmatrix} = B_t \begin{bmatrix}V_p\\ V_y\end{bmatrix},
\qquad
B_t = \begin{bmatrix}K_{pp} & K_{py}\\ K_{yp} & K_{yy}\end{bmatrix} = \begin{bmatrix}0{,}000738 & 0{,}000613\\ -0{,}000645 & 0{,}000355\end{bmatrix}\ \mathrm{N\,m/V}$$

Les quatre fonctions de transfert ont donc la forme :

$$G(s) = \begin{bmatrix}G_{11} & G_{12}\\ G_{21} & G_{22}\end{bmatrix} = \begin{bmatrix}P_p(s) & 0\\ 0 & P_y(s)\end{bmatrix} B_t,
\qquad P_p = \frac{1}{J_p s^2 + D_p s + K_{sp}},\quad P_y = \frac{1}{s\,(J_y s + D_y)}$$

> **L'idée clé :** $G_{12}$ a **la même dynamique** que $G_{11}$ (le même corps), seule l'amplitude change. Le couplage est donc **statique**, et une simple matrice de gains suffit pour l'annuler (dans le modèle).

### 1.3 Mesurer l'interaction : la RGA, en mots simples

La RGA (*Relative Gain Array*) répond à la question : *« Si je règle un correcteur sur le tangage seul, de combien son gain change-t-il quand l'autre boucle est fermée ? »*

$$\Lambda = B_t \circ \left(B_t^{-1}\right)^T = \begin{bmatrix}0{,}40 & 0{,}60\\ 0{,}60 & 0{,}40\end{bmatrix}$$

($\circ$ = produit terme à terme.) La RGA est la même à toutes les fréquences ici, puisque le couplage est statique.

- $\lambda_{11} = 1$ : aucune interaction.
- $\lambda_{11} = 0{,}40$ : quand on ferme la boucle de lacet, le gain vu par la boucle de tangage est multiplié par $1/0{,}40 = 2{,}5$. Deux PD réglés séparément ne se comportent donc pas comme prévu.
- **Vérification :** les scripts PD de Quanser, sans découpleur, donnent en réalité des pôles $-1{,}22 \pm 3{,}93j$ ($\zeta \approx 0{,}30$) au lieu de $\zeta = 0{,}64$. La réponse oscille beaucoup plus que prévu.
- $\mathrm{cond}(B_t) = 1{,}6$ : la matrice est bien conditionnée, donc on peut l'inverser sans créer de gains énormes.

---

## 2. L'idée : le découpleur $W = B_t^{-1}$

### 2.1 Le principe

Au lieu de demander des **tensions** aux correcteurs, on leur demande des **couples** $\tau_c$. Une matrice $W$ transforme ces couples en tensions :

$$V = W\,\tau_c,\qquad W = B_t^{-1}
\quad\Longrightarrow\quad
\tau_{appliqué} = B_t\,V = B_t\,B_t^{-1}\,\tau_c = \tau_c$$

Les couples appliqués sont égaux aux couples demandés : **$G_{12}$ et $G_{21}$ disparaissent**, et chaque PD ne voit plus que son propre corps.

![Schéma de principe](schema_pd_decouple.png)

### 2.2 Ce que fait $W$ physiquement

$$W = B_t^{-1} = \begin{bmatrix}540 & -933\\ 982 & 1123\end{bmatrix}\ \mathrm{V\ par\ N\,m}$$

- **Colonne 1, « je veux du couple de tangage pur » :** $V_p = 540$, $V_y = 982$ V par N·m. Les **deux** moteurs poussent. Le moteur de lacet aide à lever le nez, et son couple sur le lacet annule exactement celui du moteur de tangage.
- **Colonne 2, « je veux du couple de lacet pur » :** $V_p = -933$, $V_y = +1123$. Le moteur de tangage tourne **à l'envers**. Son couple de réaction aide le lacet, et sa poussée vers le bas annule celle du moteur de lacet sur le tangage.

> Le découpleur ne « combat » pas le couplage : il **l'utilise**. Le **dosage** $V_y/V_p$ de la colonne 1 (ici 982/540 = **1,82**) est le point crucial pour la suite (§ 7).

---

## 3. Le PD sur chaque axe

### 3.1 La loi de commande

$$\tau_p = k_{p,p}\,(r_p - \theta) - k_{d,p}\,\dot\theta + K_{sp}\,r_p
\qquad\qquad
\tau_y = k_{p,y}\,(r_y - \psi) - k_{d,y}\,\dot\psi$$

Trois choix à justifier :

1. **Dérivée sur la mesure** ($-k_d\dot\theta$), pas sur l'erreur. C'est la forme **PIV** (proportionnel-intégral-vitesse) des blocs Quanser. Un saut de consigne ne crée pas de pic de tension (« coup de dérivée »), et la boucle fermée est un 2ᵉ ordre pur, sans zéro : la réponse suit **exactement** les formules du cahier des charges. Un PID classique ($k_d\,\dot e$) ajouterait un zéro et donc plus de dépassement.
2. **Feedforward du ressort** (anticipation, bloc `SpringFF`) $K_{sp}\,r_p$ : le tangage a un ressort, donc pour tenir à 20° il faut un couple **permanent**. Un PD ne pousse que s'il y a une erreur, donc il resterait en dessous de la consigne. On ajoute directement le couple de maintien. Détails, câblage et effet mesuré : § 3.4.
$$\theta_\infty = \frac{K_{sp} + k_{p,p}}{K_{sp} + k_{p,p}}\,r_p = r_p
\qquad\text{(sans anticipation : } e_{ss} = \tfrac{K_{sp}}{K_{sp}+k_{p,p}}\,r_p = 26\,\%\text{ de la consigne)}$$
3. **Pas d'intégrateur ($K_i = 0$)** : c'est un **PD**, pas un PID. Si le jury demande « où est l'intégrale ? », la réponse est en § 11.

### 3.2 Le réglage : placement des pôles (formules de Quanser, en couple)

Sur l'axe découplé, la boucle fermée vaut :

$$J_p s^2 + (D_p + k_{d,p})\,s + (K_{sp} + k_{p,p}) = J_p\,(s^2 + 2\zeta\omega_n s + \omega_n^2)$$

On en tire directement :

$$k_{p,p} = J_p\omega_n^2 - K_{sp},\quad k_{d,p} = 2\zeta\omega_n J_p - D_p,\qquad k_{p,y} = J_y\omega_n^2,\quad k_{d,y} = 2\zeta\omega_n J_y - D_y$$

Cahier des charges de Quanser : dépassement $D$ = 7,5 % et temps de pic $t_p$ = 1,75 s.

$$\zeta = \frac{-\ln(D/100)}{\sqrt{\pi^2 + \ln^2(D/100)}} = 0{,}64,
\qquad
\omega_n = \frac{\pi}{t_p\sqrt{1-\zeta^2}} = 2{,}33\ \mathrm{rad/s}$$

| Gain | Tangage | Lacet |
|---|---|---|
| $k_p$ (N·m/rad) | 0,0878 | 0,1191 |
| $k_d$ (N·m·s/rad) | 0,0563 | 0,0536 |

Pôles en boucle fermée (modèle identifié) : $-1{,}48 \pm 1{,}80j$ sur **chaque** axe, exactement la spécification.

> **Pourquoi en N·m et pas en V ?** Ce sont les formules de `quanser_aero_pd_design_Ksp.m` multipliées par $K_{pp}$ (resp. $K_{yy}$). On règle « en physique » (couple, inertie), et c'est le découpleur qui convertit en volts. **Ne jamais relancer le script Quanser après `setup_decoupled_pd_simulink.m`** : il écraserait `kp_p` avec des valeurs en volts (107 au lieu de 0,088).

### 3.3 Même structure que le LQR

Le PD découplé complet est un **retour d'état** plus une **pré-commande** :

$$V = K_{eq}\,(x_d - x) + N_{ff}\,r,\qquad
K_{eq} = W\begin{bmatrix}k_{p,p} & 0 & k_{d,p} & 0\\ 0 & k_{p,y} & 0 & k_{d,y}\end{bmatrix} = \begin{bmatrix}47{,}4 & -111{,}0 & 30{,}4 & -50{,}0\\ 86{,}2 & 133{,}7 & 55{,}3 & 60{,}2\end{bmatrix}$$

C'est la même forme que le LQR, $K = \begin{bmatrix}98{,}2 & -103{,}1 & 32{,}3 & -29{,}1\\ 156{,}4 & 66{,}2 & 45{,}5 & 17{,}1\end{bmatrix}$, avec le **même motif de signes**. Le LQR fait lui aussi un découplage partiel, mais calculé par optimisation sur le modèle Quanser, sans contrainte explicite de découplage.


### 3.4 Le feedforward du ressort (bloc `SpringFF`) : ce qui a été implémenté dans Simulink

**Le vocabulaire.** *Feedforward*, *anticipation*, *pré-commande* et *action directe* désignent la même chose : une commande calculée **à partir de la consigne**, sans attendre qu'une erreur apparaisse. Le PD est une **rétroaction** (*feedback*) : il réagit à l'erreur. Le feedforward **anticipe** ce dont le système aura besoin.

**Pourquoi il en faut un sur le tangage.** À l'équilibre, le corps du tangage est immobile ($\dot\theta = \ddot\theta = 0$). L'équation du tangage se réduit alors à :

$$K_{sp}\,\theta = \tau_p
\qquad\Rightarrow\qquad
\tau_{maintien} = K_{sp}\,r_p = 0{,}0307 \times 0{,}349 = 10{,}7\ \mathrm{mN\,m}\ \text{ pour tenir } 20°$$

Sans feedforward, ce couple ne peut venir **que** du terme $k_{p,p}(r_p - \theta)$, donc d'une **erreur permanente** :

$$e_{ss} = \frac{K_{sp}}{K_{sp} + k_{p,p}}\,r_p = 26\,\%\ \text{de la consigne}\quad (5{,}2°\ \text{à}\ 20°)$$

Avec le feedforward, le couple de maintien est fourni **d'avance** ; le PD n'a plus qu'à corriger les écarts, et l'erreur statique est nulle dans le modèle.

**Pourquoi pas sur le lacet ?** Le lacet n'a pas de ressort ($G_{22}$ contient un intégrateur) : tenir un angle de lacet ne demande aucun couple. Il n'y a donc rien à anticiper.

**Ce qui est câblé dans Simulink.** Le bloc `SpringFF` est un Gain de valeur `Ksp` (0,0307 N·m/rad), branché **en dérivation** sur la consigne de tangage :

```
consigne θ (°) ─► D2R ─●──────────────────────────► entrée r du PIV tangage
                       │
                       └──► SpringFF (Ksp) ──► Somme (+) ◄── sortie u du PIV tangage
                                                   │
                                                   ▼  τ_p  (N·m)
                                          Mux ─► Gain W ─► Demux ─► ±20 V ─► moteurs
                                           ▲
                         sortie u du PIV lacet  (τ_y)
```

Trois points de câblage, vérifiés pendant la mise en œuvre :
- **après le bloc D2R** : la consigne doit être en **radians**, car $K_{sp}$ est en N·m/rad ;
- **en dérivation** et non **en série** : au premier câblage, le PIV recevait $K_{sp}\,r_p$ au lieu de $r_p$ et aurait visé 3 % de la consigne. Corrigé avant les essais ;
- **avant le découpleur $W$** : le feedforward est un **couple**, et c'est $W$ qui le répartit sur les deux moteurs.

**En volts, après le découpleur.** Le feedforward devient une pré-commande sur les deux moteurs :

$$V_{ff} = W\begin{bmatrix}K_{sp}\\ 0\end{bmatrix} r_p = N_{ff}\,r_p = \begin{bmatrix}16{,}6\\ 30{,}1\end{bmatrix} r_p\ \ \mathrm{(V/rad)}
\qquad\Rightarrow\qquad
V_{ff}(20°) = \begin{bmatrix}5{,}8\\ 10{,}5\end{bmatrix}\ \mathrm{V}$$

Les deux moteurs participent au maintien du tangage, avec le dosage $V_y/V_p$ = 1,82 décrit en § 2.2 et § 7.

**Ce qu'il fait réellement sur l'AERO** (paliers, dernière seconde) :

| Essai | Part du couple de maintien fournie par le feedforward | Part fournie par l'erreur du PD | Erreur statique mesurée |
|---|---|---|---|
| test1 (±10°) | 69–74 % | 26–31 % | 1,32° |
| test2 (±20°) | **87–89 %** | 11–13 % | **0,94°** |
| test3 (±20°, `cKyp` = 0,75) | 81–84 % | 16–19 % | 1,51° |

Le feedforward fait donc l'essentiel du travail. L'erreur résiduelle s'explique **exactement** par le couple manquant des moteurs (§ 8.3). Si les moteurs ne fournissent qu'une fraction $\eta$ du couple demandé, l'équilibre devient :

$$\eta\left[k_{p,p}(r_p - \theta) + K_{sp}\,r_p\right] = K_{sp}\,\theta
\quad\Rightarrow\quad
\frac{\theta}{r_p} = \frac{\eta\,(K_{sp} + k_{p,p})}{K_{sp} + \eta\,k_{p,p}}$$

| | $\eta$ mesuré | Erreur prédite par la formule | Erreur mesurée | Erreur prédite **sans** feedforward |
|---|---|---|---|---|
| test1, 10° (~5 V) | 0,63 | 1,32° | **1,32°** | 3,6° |
| test2, 20° (~9 V) | 0,84 | 0,94° | **0,94°** | 5,9° |

> **Message clé :** le feedforward divise l'erreur statique de tangage par environ 6 (5,9° → 0,94°). Ce qui reste n'est pas un défaut du PD : c'est le couple que les moteurs ne fournissent pas. Le feedforward est une compensation **en boucle ouverte** : il n'est juste que si le modèle ($K_{sp}$, gains des moteurs) est juste. Seule une **action intégrale** corrige l'erreur quelle que soit l'erreur de modèle.

**Lien avec le LQR.** L'enseignant avait suggéré la même idée pour le LQR : une pré-commande $u = K(x_{ref} - x) + N_u\,r$, qui ramenait l'erreur de 3,0° à 0,06° en simulation. L'essai LQR comparé ici (`fixed3_sys1`) n'en avait pas, d'où ses 2,7° d'erreur. **C'est la principale raison de l'écart d'erreur statique entre les deux correcteurs** : un LQR muni de $N_u$ aurait probablement une erreur du même ordre que le PD.

**Ce qui n'a pas été implémenté :** un feedforward de **vitesse** ($(k_d + D)\,\dot r$), qui supprimerait le retard de poursuite sur les rampes (§ 6.3). Testé en simulation, il donnait 15–22 % de dépassement aux coins des rampes ; il faudrait une consigne plus lisse.

---

## 4. Ce qu'on a construit

**Simulink** (à partir de `q_aero_2dof_pid_control.slx` de Quanser) :

| # | Bloc | Rôle |
|---|---|---|
| 1 | 2 blocs **PIV** ($K_i = 0$, filtre de vitesse 50 rad/s) | PD par axe, sortie = **couple** |
| 2 | Gain **`SpringFF`** (= `Ksp`) sur la consigne de tangage (en rad), ajouté à la sortie du PIV tangage | **feedforward** du ressort (§ 3.4) |
| 3 | Mux → **Gain `W`** (*Matrix(K\*u)*) → Demux | découpleur : couples → tensions |
| 4 | 2 saturations ±20 V | protection (placées **après** W) |
| 5 | Consignes *From Workspace* | rampes 20 °/s, **un seul axe bouge à la fois** |

**Essais réalisés** (rampes 20 °/s, 38 s chacun) :

| Essai | Tangage | Lacet | `cKyp` |
|---|---|---|---|
| `aero_dpd_rsp_test1` | ±10° | ±20° | 1 |
| `aero_dpd_rsp_test2` | ±20° | ±40° | 1 |
| `aero_dpd_rsp_test3` | ±20° | ±40° | **0,75** |

Vérifications faites sur les journaux :
- **unités :** plus petit pas = 1 compte codeur (0,176° / 0,088°) ;
- **`cKyp` réellement utilisé :** retrouvé à partir des tensions enregistrées, 1,00 pour test2 et **0,75** pour test3 ;
- **filtre de vitesse :** c'est bien un 2ᵉ ordre à 50 rad/s (accord à 98 % avec les tensions mesurées).

---

## 5. Les indicateurs (« indicateurs d'erreurs ») : ce qu'ils mesurent

Il n'y avait pas de cahier des charges : on a défini nos propres indicateurs, calculés **de la même façon pour tous les correcteurs**.

| Indicateur | Définition | Ce qu'il dit physiquement |
|---|---|---|
| **Erreur statique** $e_{ss}$ | $\lvert r - \bar y\rvert$ moyennée sur la **dernière seconde d'un palier** | le correcteur atteint-il la cible ? (ressort, frottement, couple insuffisant) |
| **Dépassement** | $100\,(\max z - 1)$, avec $z$ = mouvement normalisé | amortissement |
| **RMSE** | $\sqrt{\frac{1}{N}\sum (r_k - y_k)^2}$ sur l'essai | erreur globale, **mais dépend de la consigne** (voir le piège ci-dessous) |
| **Couplage** | $\max\lvert y_j(t) - y_j(t_0)\rvert$ pendant qu'**un seul** axe bouge | ce que le mouvement d'un axe fait à l'autre |
| **Saturation** | % du temps à ±20 V | la boucle reste-t-elle linéaire ? |
| **Activité de commande** | $\frac{1}{T}\sum_k \lvert\Delta V_{p,k}\rvert + \lvert\Delta V_{y,k}\rvert$ (V/s) | bruit de dérivation des codeurs, usure moteurs |
| **Fit** | $100\left(1 - \frac{\lVert y - \hat y\rVert}{\lVert y - \bar y\rVert}\right)$ | le modèle prédit-il la mesure ? |
| **Couple obtenu / demandé** $\eta$ | $\dfrac{K_{sp}\,\bar\theta}{K_{pp}\bar V_p + K_{py}\bar V_y}$ sur un palier | les moteurs donnent-ils le couple prévu par le modèle ? |

> **Piège de la RMSE.** L'essai LQR (`fixed3_sys1`) a 6 rampes de tangage en 30 s, et le lacet bouge parfois en même temps. L'essai PD n'a que 3 rampes en 38 s, un axe à la fois. Plus il y a de rampes, plus la RMSE est grande, quel que soit le correcteur. **La RMSE globale n'est donc pas comparable entre ces deux essais.** Le tableau du plan (diapo 13 : « RMSE 3,9/5,3 contre 2,8/4,0 ») est à **remplacer** par la comparaison du § 6.3.

> **Mesurer le couplage exige qu'un seul axe bouge.** Sinon on ne sait pas qui a perturbé qui. C'est pour ça que les consignes PD sont décalées. Pour le LQR, c'est seulement possible sur certains mouvements de l'essai `fixed3`.

---

## 6. Résultats

### 6.1 En simulation : le découplage fonctionne

Modèle « réaliste » : paramètres avec frottement sec, retards moteurs (50/100 ms), quantification des codeurs, filtre de vitesse. Même consigne que test2.

| (simulation) | PD découplé | LQR Current |
|---|---|---|
| $e_{ss}$ tangage | 0,1° | 2,7° |
| Lacet déplacé par le tangage | 0,5° | 2,2° |
| Tangage déplacé par le lacet | 0,5° | 1,9° |

### 6.2 Sur l'AERO : la mesure suit la simulation

![Suivi test2](fig_pd_1_suivi.png)

- **Fit simulation/mesure : 92 % (tangage), 97 % (lacet)** : le modèle identifié prédit bien le comportement.
- 0 % de saturation (tension max 15,7 V).
- Écart principal : **le tangage s'arrête ~1° avant la consigne** sur les paliers à ±20°, alors que la simulation l'atteint. La raison est en § 8.3.

### 6.3 Comparaison honnête avec le LQR (mesures)

On compare **ce qui est comparable** :
- les **paliers** (tangage ±20°, lacet ±40°) ;
- **la même rampe de tangage +20° → −20° avec le lacet immobile**, présente dans les deux essais (LQR : t = 20 s ; PD : t = 13 s) ;
- le **début (2,5 s) d'une rampe de lacet de 80°** avec le tangage immobile.

![Même rampe](fig_pd_2_meme_rampe.png)

![Indicateurs](fig_pd_3_indicateurs.png)

| Indicateur (mesuré) | LQR Current | PD, `cKyp` = 1 (test2) | PD, `cKyp` = 0,75 (test3) |
|---|---|---|---|
| $e_{ss}$ tangage, paliers ±20° | 2,74° | **0,94°** | 1,51° |
| $e_{ss}$ lacet, paliers ±40° | 0,78° | 1,27° | 0,78° |
| Même rampe : erreur finale du tangage | 3,4° | **0,9°** | 1,6° |
| Même rampe : retard (passage à −16°) | **0,38 s** | 0,44 s | 0,46 s |
| Lacet déplacé par cette rampe | **1,3°** | 2,7° | 1,7° |
| Tangage déplacé (2,5 s de rampe de lacet) | **0,9°** | 1,1° | 1,2° |
| Saturation | 0 % | 0 % | 0 % |
| Activité de commande | 107 V/s | 97 V/s | 107 V/s |

**Lecture, en trois phrases pour la diapo :**
1. **Précision : le PD gagne.** L'erreur statique de tangage est divisée par 3 grâce au feedforward du ressort (`SpringFF`) ; l'essai LQR n'avait pas de pré-commande $N_u$.
2. **Rapidité : le LQR est un peu plus rapide** (0,38 s contre 0,44 s), car il est plus « raide » sur le tangage (raideur équivalente 0,17 N·m/rad, soit $\omega_n \approx 2{,}8$ rad/s, contre 0,12 N·m/rad et 2,33 rad/s pour le PD). Le PD a été réglé avec les spécifications de Quanser ; un $t_p$ = 1,5 s le rendrait aussi rapide.
3. **Couplage : pas de gain réel.** La simulation promettait un couplage 3 à 4 fois plus faible ; sur la maquette, il est du même ordre que le LQR, voire un peu plus fort tant que `cKyp` n'est pas réglé. Les causes sont en § 8.

> Hypothèse : le LQR « fait mieux » en couplage presque par chance : son dosage $V_y/V_p$ pour tenir le tangage vaut 156/98 = 1,59, plus proche du dosage réel sans fuite que celui de notre découpleur (1,82). Voir § 7.

---

## 7. Le réglage `cKyp`, expliqué simplement

### 7.1 Ce que c'est

`cKyp` est un **facteur de correction** sur un seul paramètre du découpleur :

$$W = \begin{bmatrix}K_{pp} & K_{py}\\ c_{yp}\,K_{yp} & K_{yy}\end{bmatrix}^{-1}$$

Il change le **dosage** des deux moteurs quand on demande du couple de tangage :

| `cKyp` | Dosage $V_y/V_p$ pour un couple de tangage pur | Tensions pour tenir 20° ($V_p$ / $V_y$) |
|---|---|---|
| 1 | 1,82 | 5,8 / 10,5 V |
| 0,75 | 1,37 | 6,8 / 9,3 V |
| 0,5 | 0,91 | 8,3 / 7,5 V |

### 7.2 Pourquoi on l'a touché

Pour que tenir le tangage ne crée **aucun couple sur le lacet**, il faut :

$$\tau_y^{parasite} = \left(K_{yp}^{r}\,W_{11} + K_{yy}^{r}\,W_{21}\right)\tau_p = 0
\quad\Longleftrightarrow\quad
\frac{V_y}{V_p} = \frac{W_{21}}{W_{11}} = -\frac{K_{yp}^{r}}{K_{yy}^{r}}$$

où l'exposant $r$ désigne les gains **réels**. Le modèle donne 1,82. Mais si le vrai rapport est différent, **une partie du couple de maintien du tangage « fuit » dans le lacet**.

**Symptôme dans test2 :**
- après chaque mouvement de tangage, une fois le tangage stabilisé, le lacet dérive lentement **dans le sens du tangage** : −2,7° après la descente de 40° ;
- sur les paliers, l'erreur du lacet suit le **signe** du tangage.

Le lacet ne dérive que quand la fuite dépasse le frottement sec. C'est donc un effet **statique** (lié au couple de maintien), pas dynamique.

Le modèle a de bonnes raisons de se tromper ici : $K_{yp}$ vient de l'essai G21 et $K_{yy}$ de G22, et l'ajustement avec frottement a trouvé un $K_{yy}$ deux fois plus grand que CONTSID. Le **rapport** $K_{yp}/K_{yy}$ est donc incertain.

### 7.3 Résultat (test3, `cKyp` = 0,75)

- Dérive du lacet après la rampe de 40° : **−2,7° → −1,6°** (−40 %).
- Erreur statique du lacet : 1,27° → 0,78°.
- **Prix à payer :** l'erreur statique de tangage passe de 0,9° à 1,5°. Le couple de tangage est davantage fourni par le moteur de tangage, qui se révèle moins efficace que prévu (couple obtenu 84 % → 77 %, § 8.3).

### 7.4 Ce qui reste

- La dérive a toujours le même signe, donc **le bon réglage est plus bas, vers 0,5–0,6** (essai non réalisé, faute de temps maquette).
- Les **bosses du tangage** au début et à la fin d'une rampe de lacet n'ont **pas changé** entre test2 et test3 : elles ne dépendent pas des gains statiques (§ 8.2). C'est pourquoi on n'a pas touché `cKpy`.

---

## 8. Les limites physiques : ce que le PD découplé ne peut pas corriger

| # | Symptôme mesuré | Cause physique | Pourquoi le PD ne le corrige pas | Remède |
|---|---|---|---|---|
| 1 | Le lacet dérive quand le tangage est tenu | Rapport réel $K_{yp}/K_{yy}$ ≠ modèle | Le découpleur est **calculé**, pas mesuré | Régler `cKyp` (0,5–0,6), ou identifier les gains croisés en boucle fermée |
| 2 | Bosses du tangage (+0,9° / −2,1°) au début et à la fin des rampes de lacet | **Couple de réaction** quand l'hélice de lacet accélère ou freine (inertie du rotor) | Effet **dynamique** : un gain statique ne peut pas l'annuler (G12 n'avait que 60 % de fit pour la même raison) | Rampes plus douces, ou terme dynamique utilisant la vitesse mesurée des hélices |
| 3 | Le tangage s'arrête 1–1,5° avant la consigne | Les moteurs donnent **moins de couple que prévu** : poussée en $V^2$ à basse tension, et gains du modèle trop optimistes | Le feedforward (`SpringFF`) suppose le gain du modèle | **Action intégrale** (PID découplé, $K_i \neq 0$) |
| 4 | Lacet bloqué à ~1° de la cible | **Frottement sec** : bande morte $\lvert e_\psi\rvert \le F_{c,y}/k_{p,y} \approx 2°$ | Un PD ne pousse pas assez près de la cible pour décoller | Intégrateur sur le lacet (attention au stick-slip) |
| 5 | Activité 97–107 V/s (simulation : 50) | Dérivée de codeurs quantifiés, plus les vibrations | Inhérent à la dérivation | Observateur (Luenberger / Kalman) |
| 6 | (en échelons) le tangage tombe de 24° pendant un échelon de lacet | **Saturation** : quand un moteur sature, $B_t W \neq I$ | Le découplage n'existe que hors saturation | Rampes (0 % de saturation), ou saturation « priorité tangage » (simulée seulement) |

### 8.3 Zoom sur la limite n° 3 : le couple manquant

![Couple obtenu / demandé](fig_pd_4_couple.png)

Sur un palier, le tangage est immobile, donc **couple obtenu = couple du ressort** $K_{sp}\theta$. On le compare au couple que le modèle attribue aux tensions mesurées :

- **test1, ~5 V : 63 %.** La poussée en $V^2$ explique presque tout : le couple par volt est proportionnel à $|V|$, donc à 5 V on a environ 5/8 = 62 % du couple à 8 V, la tension d'identification.
- **test2, ~9 V : 84 %.** Ici $V^2$ devrait donner *plus* que 100 %. Les 16 % manquants viennent donc d'une **erreur de gain du modèle** (hypothèse : $K_{pp}$ ou $K_{py}$ surestimés ; $K_{py}$ est le paramètre le moins sûr).
- **test3 : 77 %.** Avec `cKyp` = 0,75, le moteur de tangage travaille plus, et il est moins efficace que prévu.

> **Message pour le jury :** l'erreur statique résiduelle du PD (≈ 1°) a la même origine que celle du LQR (≈ 3°) : les moteurs ne fournissent pas le couple que le modèle linéaire attend. Le PD la réduit grâce au feedforward ; seule une **action intégrale** l'annule quelle que soit l'erreur de modèle.

---

## 9. Quoi montrer, diapo par diapo (mise à jour du plan)

### Diapo 12 — PD découplé : principe

- **Titre (message clé) :** *Tout le couplage est dans les moteurs : on l'inverse, puis on règle un PD par axe.*
- **Figure :** `schema_pd_decouple.png`.
- **Formules :** F2 (Bt), F4 (découpleur), F7 (loi PD avec feedforward), F9 (gains) du § 10. Si vous avez de la place : F18 (feedforward en volts).
- **Chiffres :** RGA $\lambda_{11}$ = 0,40 (forte interaction), $\zeta$ = 0,64, $\omega_n$ = 2,33 rad/s.
- **À dire :** *« Le modèle montre que les deux corps sont indépendants : seuls les moteurs mélangent les axes. En inversant ce mélange, chaque PD ne voit que son axe, et on le règle avec les formules classiques du second ordre. On ajoute un feedforward : le couple de maintien du ressort, calculé à partir de la consigne, pour supprimer l'erreur statique. »*

### Diapo 13 — PD découplé : résultats et comparaison avec le LQR

- **Titre :** *Le PD divise l'erreur statique par 3, mais le découplage réel est limité par le modèle.*
- **Figure principale :** `fig_pd_2_meme_rampe.png`, ou `fig_pd_3_indicateurs.png` si vous préférez des barres.
- **Chiffres :** $e_{ss}$ tangage 0,9° (PD) contre 2,7° (LQR) ; fit 92/97 % ; couplage 1,7–2,7° (PD) contre 1,3° (LQR) ; 0 % de saturation.
- **À dire :** *« Sur le même mouvement, le PD atteint presque la consigne grâce au feedforward du ressort, qui fournit près de 90 % du couple de maintien, alors que le LQR reste 3° au-dessus. En simulation, le couplage était 4 fois plus faible ; sur la maquette, il est du même ordre que le LQR, car le découpleur repose sur les gains croisés, les moins bien identifiés, et ne peut pas compenser les coups de couple des hélices. Un réglage du découpleur (`cKyp`) a déjà réduit la dérive du lacet de 40 %. »*

### Annexes (pour les questions)

- `fig_pd_1_suivi.png` : simulation vs mesure.
- `fig_pd_4_couple.png` : couple manquant.
- Tableau du § 7.1 (`cKyp`).
- Tableau du § 8 (limites).

### Corrections à faire dans le plan

- **Diapo 13 :** remplacer le tableau par celui du § 6.3. Supprimer la comparaison de RMSE globale (non comparable, § 5).
- **Diapo 14 :** « PD découplé : erreur statique ÷2, RMSE −25 % » devient « **erreur statique de tangage ÷3 ; couplage du même ordre que le LQR** ».
- **Diapo 13 :** « test2/test3 donnent la dispersion d'un essai à l'autre » est **faux** : test3 n'a pas le même réglage (`cKyp` = 0,75). La différence entre les deux est l'**effet de `cKyp`**, pas une dispersion.
- **Diapo 15 :** ajouter « réglage fin du découpleur (`cKyp` ≈ 0,5–0,6) » et « compensation dynamique du couple de réaction des hélices ».

---

## 10. Formules LaTeX pour PowerPoint

**Mode d'emploi :** dans PowerPoint, faites **Insertion → Équation**, choisissez **LaTeX** dans le ruban, collez le texte (sans `$$`), puis cliquez sur **Convertir → Professionnel**. Testez d'abord une formule avec matrice ; si une matrice ne se convertit pas, insérez-la avec **Équation → Matrice**.

**F1 — Modèle (structure Quanser)**
```
J_p\ddot{\theta}+D_p\dot{\theta}+K_{sp}\theta=K_{pp}V_p+K_{py}V_y
```
```
J_y\ddot{\psi}+D_y\dot{\psi}=K_{yp}V_p+K_{yy}V_y
```

**F2 — Le couplage est dans les moteurs**
```
\left[\begin{matrix}\tau_p\\\tau_y\end{matrix}\right]=B_t\left[\begin{matrix}V_p\\V_y\end{matrix}\right],\qquad B_t=\left[\begin{matrix}K_{pp}&K_{py}\\K_{yp}&K_{yy}\end{matrix}\right]
```

**F3 — Fonctions de transfert**
```
G\left(s\right)=\left[\begin{matrix}P_p\left(s\right)&0\\0&P_y\left(s\right)\end{matrix}\right]B_t,\qquad P_p=\frac{1}{J_ps^2+D_ps+K_{sp}},\qquad P_y=\frac{1}{s\left(J_ys+D_y\right)}
```

**F4 — Découpleur**
```
V=W\,\tau_c,\qquad W=B_t^{-1}\quad\Rightarrow\quad G\left(s\right)W=\left[\begin{matrix}P_p&0\\0&P_y\end{matrix}\right]
```

**F5 — Découpleur numérique (V par N·m)**
```
W=B_t^{-1}=\left[\begin{matrix}540&-933\\982&1123\end{matrix}\right]
```

**F6 — RGA**
```
\Lambda=B_t\circ\left(B_t^{-1}\right)^T=\left[\begin{matrix}0{,}40&0{,}60\\0{,}60&0{,}40\end{matrix}\right]
```

**F7 — Loi de commande PD (dérivée sur la mesure, feedforward du ressort)**
```
\tau_p=k_{p,p}\left(r_p-\theta\right)-k_{d,p}\dot{\theta}+K_{sp}r_p
```
```
\tau_y=k_{p,y}\left(r_y-\psi\right)-k_{d,y}\dot{\psi}
```

**F8 — Boucle fermée d'un axe**
```
J_ps^2+\left(D_p+k_{d,p}\right)s+\left(K_{sp}+k_{p,p}\right)=J_p\left(s^2+2\zeta\omega_ns+\omega_n^2\right)
```

**F9 — Gains**
```
k_{p,p}=J_p\omega_n^2-K_{sp},\quad k_{d,p}=2\zeta\omega_nJ_p-D_p,\quad k_{p,y}=J_y\omega_n^2,\quad k_{d,y}=2\zeta\omega_nJ_y-D_y
```

**F10 — Spécifications**
```
\zeta=\frac{-\ln\left(D/100\right)}{\sqrt{\pi^2+\ln^2\left(D/100\right)}}=0{,}64,\qquad\omega_n=\frac{\pi}{t_p\sqrt{1-\zeta^2}}=2{,}33\ rad/s
```

**F11 — Rôle du feedforward du ressort**
```
\theta_{\infty}=\frac{K_{sp}+k_{p,p}}{K_{sp}+k_{p,p}}r_p=r_p\qquad\left(sans\ :\ e_{ss}=\frac{K_{sp}}{K_{sp}+k_{p,p}}r_p\approx0{,}26\,r_p\right)
```

**F12 — Forme « retour d'état » (comme le LQR)**
```
V=K_{eq}\left(x_d-x\right)+N_{ff}\,r,\qquad K_{eq}=W\left[\begin{matrix}k_{p,p}&0&k_{d,p}&0\\0&k_{p,y}&0&k_{d,y}\end{matrix}\right]
```

**F13 — Réglage cKyp : condition « pas de fuite vers le lacet »**
```
W=\left[\begin{matrix}K_{pp}&K_{py}\\c_{yp}K_{yp}&K_{yy}\end{matrix}\right]^{-1},\qquad\frac{V_y}{V_p}=\frac{W_{21}}{W_{11}}=-\frac{K_{yp}^{r}}{K_{yy}^{r}}
```

**F14 — Indicateurs**
```
e_{ss}=\left|r-\bar{y}\right|_{derniere\ seconde\ du\ palier},\qquad RMSE=\sqrt{\frac{1}{N}\sum_{k=1}^{N}\left(r_k-y_k\right)^2}
```
```
fit=100\left(1-\frac{\left\|y-\hat{y}\right\|}{\left\|y-\bar{y}\right\|}\right),\qquad activite=\frac{1}{T}\sum_k\left(\left|\Delta V_{p,k}\right|+\left|\Delta V_{y,k}\right|\right)
```

**F15 — Couple obtenu / demandé (sur un palier)**
```
\eta=\frac{K_{sp}\bar{\theta}}{K_{pp}\bar{V}_p+K_{py}\bar{V}_y}
```

**F16 — Limites physiques**
```
\tau\propto V\left|V\right|\quad\Rightarrow\quad\frac{d\tau}{dV}\propto\left|V\right|
```
```
\left|e_{\psi}\right|\le\frac{F_{c,y}}{k_{p,y}}\approx2^{\circ},\qquad\left|e_{\theta}\right|\le\frac{F_{c,p}}{K_{sp}+k_{p,p}}\approx0{,}3^{\circ}
```

**F17 — Retard de poursuite sur une rampe (forme PV)**
```
e_{rampe}=\frac{D_p+k_{d,p}}{K_{sp}+k_{p,p}}\,\dot{r}=\frac{2\zeta}{\omega_n}\dot{r}\approx0{,}55\ s\times\dot{r}
```

**F18 — Le feedforward (bloc SpringFF) en couple puis en volts**
```
\tau_{ff}=K_{sp}\,r_p,\qquad V_{ff}=W\left[\begin{matrix}K_{sp}\\0\end{matrix}\right]r_p=\left[\begin{matrix}16{,}6\\30{,}1\end{matrix}\right]r_p
```

**F19 — Erreur résiduelle quand les moteurs ne donnent qu'une fraction η du couple**
```
\frac{\theta}{r_p}=\frac{\eta\left(K_{sp}+k_{p,p}\right)}{K_{sp}+\eta\,k_{p,p}}\qquad\left(\eta=0{,}84\ \Rightarrow\ e_{ss}=0{,}94^{\circ}\ a\ 20^{\circ}\right)
```

**F20 — Pré-commande équivalente du LQR (suggestion de l'enseignant)**
```
u=K\left(x_{ref}-x\right)+N_u\,r
```

---

## 11. Questions probables du jury (PD) et réponses courtes

1. **« Pourquoi un découpleur statique suffit-il ? »** Dans le modèle de Quanser, les deux corps sont indépendants ; seuls les moteurs mélangent les couples (matrice $B_t$, constante). $G_{12}$ a la même dynamique que $G_{11}$ : inverser $B_t$ annule le couplage à toutes les fréquences.
2. **« Pourquoi le couplage mesuré n'est-il pas meilleur que celui du LQR ? »** Le découpleur repose sur $K_{py}$ et $K_{yp}$, les paramètres les moins bien identifiés (G12 : 60 % de fit). Il est statique alors qu'une partie du couplage est dynamique (couple de réaction des hélices pendant leur accélération). Et le gain réel des moteurs dépend de la tension ($V^2$).
3. **« Où est l'intégrateur ? »** Il n'y en a pas : c'est un PD (bloc PIV avec $K_i = 0$). L'erreur statique a été réduite par l'anticipation du ressort, qui suffit dans le modèle. L'erreur restante (≈ 1°) vient du couple manquant des moteurs ; l'étape suivante est $K_i \neq 0$ dans le même bloc (PID découplé).
4. **« Pourquoi la forme PIV et pas PID ? »** La dérivée porte sur la mesure : pas de pic de tension quand la consigne change, et pas de zéro en boucle fermée, donc la réponse correspond exactement à $\zeta$ et $\omega_n$ choisis.
5. **« Comment avez-vous choisi les gains ? »** Par placement de pôles sur chaque axe découplé, avec la spécification de Quanser (7,5 %, 1,75 s), en unités de couple, avec les paramètres identifiés par CONTSID.
6. **« À quoi sert `cKyp` ? »** C'est une correction du gain croisé $K_{yp}$ dans le découpleur. Il règle le dosage des deux moteurs pour tenir le tangage sans pousser le lacet. Le passer de 1 à 0,75 a réduit la dérive du lacet de 40 %.
7. **« Comment êtes-vous sûrs que `cKyp` valait bien 0,75 pendant l'essai ? »** On l'a retrouvé à partir des tensions enregistrées : en reconstruisant les couples des PD et en résolvant $\tau = B_t V$ par moindres carrés, on trouve 1,00 pour test2 et 0,75 pour test3.
8. **« Pourquoi comparer sur un seul mouvement et pas sur tout l'essai ? »** Les deux essais n'ont pas la même séquence de consignes : la RMSE globale dépend du nombre de rampes. On compare donc des paliers et une rampe identique présente dans les deux essais.
9. **« Le LQR est-il plus rapide ? »** Légèrement (0,38 s contre 0,44 s de retard) : il est plus raide sur le tangage. Le PD a été réglé avec les spécifications de Quanser ; un temps de pic de 1,5 s le rendrait aussi rapide, au prix d'un peu plus de bruit.
10. **« Que se passe-t-il si un moteur sature ? »** Le découplage disparaît pendant la saturation : en échelons, la simulation montre une chute de 24° du tangage. Les rampes évitent la saturation (0 %). Une allocation « priorité tangage » (on réduit seulement le couple de lacet) garde le découplage ; elle a été simulée mais pas testée sur la maquette.
11. **« Qu'est-ce que le bloc SpringFF ? »** Un feedforward : le couple nécessaire pour tenir le ressort du tangage à l'angle demandé, $K_{sp}\,r_p$, calculé à partir de la consigne et ajouté à la sortie du PD avant le découpleur. Il fournit près de 90 % du couple de maintien ; le PD ne corrige que le reste.
12. **« Pourquoi pas de feedforward sur le lacet ? »** Le lacet n'a pas de ressort : tenir un angle ne demande aucun couple.
13. **« Le feedforward suffit-il ? »** Non : c'est une compensation en boucle ouverte, juste seulement si le modèle est juste. Il divise l'erreur par 6 (5,9° → 0,94°) ; l'erreur restante correspond exactement au couple que les moteurs ne fournissent pas (formule F19). Un intégrateur corrigerait le reste.
14. **« Pourquoi le LQR a-t-il plus d'erreur statique ? »** L'essai LQR comparé n'avait pas de pré-commande $N_u$. Avec la même idée (suggérée par l'enseignant, validée en simulation), l'écart d'erreur statique disparaîtrait probablement.

---

## 12. Fichiers

| Fichier | Rôle |
|---|---|
| `figures_pd_presentation_AERO.m` | **Les 4 figures de la présentation** + tous les chiffres de ce guide |
| `schema_pd_decouple.png` | Schéma de principe (diapo 12) |
| `decoupled_pd_AERO.m` | Conception (gains, W, RGA) et simulation de plusieurs variantes |
| `setup_decoupled_pd_simulink.m` | Variables du modèle Simulink (InitFcn) |
| `plot_decoupled_pd_tests_AERO.m` | Dépouillement détaillé des essais (vérification de `cKyp`, etc.) |
| `rapport_pd_decouple_AERO.md` | Rapport de conception (version avant les essais) |

> **Note sur la vérification.** MATLAB n'étant pas disponible de mon côté, les scripts ont été vérifiés par une reproduction en Python des mêmes calculs, sur les mêmes fichiers. Les images fournies viennent de cette reproduction. Lancez `figures_pd_presentation_AERO.m` une fois ce soir : les chiffres affichés doivent correspondre à ceux de ce guide, aux arrondis près.

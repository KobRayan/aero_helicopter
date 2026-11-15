# Guide : modèle de l'AERO, linéarité et non-linéarités

*Mini-projet Quanser AERO, Polytech Nancy. Préparation de la soutenance du 16/10/2026.*

Ce guide reprend tout le travail sur le **modèle** fait dans cette conversation : identification CONTSID, validation, frottement sec et test de linéarité en impulsions 0 → X V. Il complète `plan_presentation_AERO.md`, qui reste le plan de la soutenance.

Il couvre surtout les **diapos 4, 5, 9, 10 et 11** de ce plan. Le § 7 trie tous les fichiers `.m` du projet. Le § 8 dit quelles figures montrer demain.

---

## 1. Qui est qui : notations (à maîtriser avant tout)

### 1.1 Entrées, sorties, état

- **Entrées :** $V_p$ (tension du moteur de tangage) et $V_y$ (tension du moteur de lacet), saturées à ±20 V.
- **Sorties :** $\theta$ (tangage) et $\psi$ (lacet), en rad dans les modèles et en degrés dans les fichiers.
- **État :** $x = [\theta,\ \psi,\ \dot\theta,\ \dot\psi]^T$, avec $T_e = 2$ ms.

### 1.2 Les 4 fonctions de transfert

$G_{ij}$ = sortie *i* / entrée *j* (1 = tangage, 2 = lacet) :

| | Entrée $V_p$ (moteur de tangage) | Entrée $V_y$ (moteur de lacet) |
|---|---|---|
| **Sortie $\theta$** | $G_{11}$ : effet **direct** | $G_{12}$ : **couplage** (lacet → tangage) |
| **Sortie $\psi$** | $G_{21}$ : **couplage** (tangage → lacet) | $G_{22}$ : effet **direct** |

$$\begin{bmatrix}\theta\\ \psi\end{bmatrix} = \begin{bmatrix}G_{11}(s) & G_{12}(s)\\ G_{21}(s) & G_{22}(s)\end{bmatrix}\begin{bmatrix}V_p\\ V_y\end{bmatrix}$$

> ⚠ La figure du Lab Guide range les fonctions **par entrée**, donc dans l'ordre transposé. Si le jury pose la question : « $G_{12}$, c'est l'effet de $V_y$ sur $\theta$ ».

### 1.3 Le modèle de Quanser (Lab Guide 2 DDL, éq. 2.1 à 2.4, § 2.1.2)

C'est la structure **imposée par l'enseignant**. On ne l'a pas modifiée : on a seulement ré-identifié ses paramètres.

$$J_p\ddot\theta + D_p\dot\theta + K_{sp}\theta = K_{pp}V_p + K_{py}V_y$$
$$J_y\ddot\psi + D_y\dot\psi = K_{yp}V_p + K_{yy}V_y$$

$$G_{11} = \frac{K_{pp}}{J_p s^2 + D_p s + K_{sp}},\quad G_{12} = \frac{K_{py}}{J_p s^2 + D_p s + K_{sp}},\quad G_{21} = \frac{K_{yp}}{s(J_y s + D_y)},\quad G_{22} = \frac{K_{yy}}{s(J_y s + D_y)}$$

Lecture physique de ces équations :

- **Tangage :** un ressort ($K_{sp}$) ramène le corps à l'horizontale, donc c'est un **2ᵉ ordre stable**.
- **Lacet :** pas de ressort, donc il a un **intégrateur**. Une tension constante donne une vitesse constante, et l'angle croît sans fin : c'est un **1ᵉʳ ordre + intégrateur**.
- **Partage des corps :** $G_{11}$ et $G_{12}$ ont le même dénominateur (le même « corps »), tout comme $G_{21}$ et $G_{22}$. Seuls les gains changent.

En représentation d'état (`quanser_aero_state_space.m`), $A$ contient les deux « corps » ($K_{sp}$, $D_p$, $D_y$) et $B$ contient les 4 gains de couple :

$$A = \begin{bmatrix}0&0&1&0\\0&0&0&1\\-\frac{K_{sp}}{J_p}&0&-\frac{D_p}{J_p}&0\\0&0&0&-\frac{D_y}{J_y}\end{bmatrix},\qquad B = \begin{bmatrix}0&0\\0&0\\\frac{K_{pp}}{J_p}&\frac{K_{py}}{J_p}\\\frac{K_{yp}}{J_y}&\frac{K_{yy}}{J_y}\end{bmatrix}$$

$J_p = 0{,}02189$ et $J_y = 0{,}02199$ kg·m² sont **conservés** (géométrie). Les 7 autres paramètres sont identifiés.

---

## 2. Pourquoi revoir le modèle ? (le point de départ)

Le LQR synthétisé sur le modèle Quanser **fonctionne**, mais la maquette s'écarte de la simulation de façon systématique :

- **Erreur statique de tangage :** environ **2× la simulation**, pour les 4 réglages.
- **Dépassement du lacet :** 7–12 % mesurés contre 1–2 % simulés.
- **Couplage :** plus fort que prévu.

L'enseignant a donc demandé de **réviser le modèle** : identifier $G_{11}$, $G_{12}$, $G_{21}$, $G_{22}$ avec la boîte à outils **CONTSID**, en gardant la structure Quanser.

---

## 3. Ce qu'on a fait, dans l'ordre, et pourquoi

### 3.1 Premier essai : identification sur les échelons (abandonné)

Script : `identify_steps_AERO.m`.

- **Méthode :** `tfsrivc` avec des ordres libres, sur les échelons de 40 s (5/10/15 V). Pour le lacet, on identifiait la **vitesse** puis on ajoutait $1/s$.
- **Limites :**
  - une seule amplitude par essai ;
  - le lacet tourne très vite à 10–15 V, donc l'effet centrifuge et la poussée en V² apparaissent ;
  - les ordres libres ne respectent pas la structure Quanser.
- **Décision :** l'enseignant a demandé de garder la structure du Lab Guide, donc on est passé à `identify_quanser_AERO.m`.

### 3.2 Nouveaux essais d'identification (créneaux à 8 V)

Les essais ont été faits sous Simulink `AERO_basics.slx` : `data0` à 12 lignes, signaux construits avec `Repeating Sequence Stair` + `repelem`.

| Fichier | Conditions | Signal | Sert à |
|---|---|---|---|
| `custom_pitch_free_8V` | libre | $V_p$ : 0 / −8 / 0 / +8 V, 10 s par palier | **G11** |
| `custom_yaw_free_8V` | libre | $V_y$ : 0 / −8 / 0 / +8 V | **G12 et G22** |
| `step_pitch_10V` | libre | échelon $V_p$ = 10 V | **G21** |
| `custom_pitch_8V_blocked`, `custom_yaw_blocked_8V` | lacet bloqué | idem | comparaison (même résultat à 8 V) |
| `custom_pitchsteps_8V_locked` | lacet bloqué | escalier $V_p$ 0, 3, 6, 9, 12… −12 V | validation, V² |
| `custom_freeramp_pitch` | libre | $V_p$ 0 → 10 V par pas de 1 V / 10 s | validation |
| `custom_free_2dof1/2` | libre | $V_p$ ±6 V et $V_y$ ±9 V, périodes différentes | **validation 2 DDL** (0–88 s seulement) |

Remarques utiles si on vous interroge sur le protocole :

- **Pourquoi 8 V ?** Au-delà de la zone morte du lacet (6–7 V), mais assez bas pour que le lacet tourne lentement (≈ 22 °/s max). L'effet centrifuge reste alors négligeable.
- **Pourquoi des paliers de 10 s ?** Le tangage a le temps de se stabiliser (période ≈ 5 s).
- **Le tangage ne se bloque pas** sur notre AERO (le tenir à la main fausse tout). Seul le lacet se bloque, avec la clé Allen.
- **L'enseignant :** « bloquer le lacet ne représente pas le système réel ». On a donc retenu les **essais libres**. À 8 V, libre et bloqué donnent le même $G_{11}$ / $G_{12}$.
- **Essais 2 DDL instables vers 91 s :** les deux tensions sont devenues négatives à moins d'1 s d'intervalle. Le tangage a plongé à −55° et a touché la base, et QUARC s'est arrêté. On n'utilise que 0–88 s.
- **Erreur Simulink « multi-rate » :** un bloc à 10 s dans un modèle à 2 ms. Corrigé avec `repelem(..., 10/0.002)` et une période de 0,002 s.

### 3.3 Identification CONTSID avec la structure Quanser

Script : `identify_quanser_AERO.m`.

| Axe | Fonction CONTSID | Modèle obtenu |
|---|---|---|
| Tangage ($G_{11}$, $G_{12}$) | `tfsrivc(data, 2, 0, 'TdMax', TdMax)` | $\dfrac{b_0}{s^2 + a_1 s + a_0}\,e^{-T_d s}$ (2 pôles, 0 zéro, retard) |
| Lacet ($G_{21}$, $G_{22}$) | `procsrivc(data, 'P1I')` | $\dfrac{K}{s(1+Ts)}$ (1ᵉʳ ordre + intégrateur) |

- **Pourquoi deux fonctions ?** L'enseignant l'a précisé : `procsrivc` ne gère que les modèles du 1ᵉʳ ordre, `P1D` ($K e^{-T_d s}/(1+Ts)$) et `P1I` ($K/(s(1+Ts))$). Il est idéal pour le lacet. Pour le 2ᵉ ordre du tangage, il faut `tfsrivc`.
- **SRIVC** (*Simplified Refined Instrumental Variable for Continuous-time*) : identification directe d'un modèle **à temps continu**. Les paramètres sont donc physiques, et la méthode est robuste au bruit de mesure.
- **Réglages :** `iddata(y, u, Ts, 'InterSample', 'zoh')`, car la tension est constante entre deux échantillons (HIL Write).

Passage des coefficients aux paramètres physiques :

$$K_{sp} = J_p a_0,\quad D_p = J_p a_1,\quad K_{pp} = J_p b_0,\quad K_{py} = K_{12}^{dc}\,K_{sp},\quad D_y = \frac{J_y}{T},\quad K_{yy} = K_{22}\,D_y,\quad K_{yp} = K_{21}\,D_y$$

(Pour $G_{12}$ et $G_{21}$ on ne garde que le **gain**, car ils partagent le corps de $G_{11}$ / $G_{22}$.)

**Retard :** `TdMax = 0,1` donnait $T_d = 0{,}100$ s, c'est-à-dire en butée. Avec `TdMax = 0,2`, on obtient $T_d = 0{,}117$ s. Le retard est mal déterminé (0,05–0,12 s : moteur + montée en vitesse de l'hélice), mais il **ne change pas A et B**. Il n'est pas dans le modèle d'état à 4 états.

**Fits** sur l'essai d'identification, avec $\text{fit} = 100\left(1 - \frac{\lVert y - \hat y\rVert}{\lVert y - \bar y\rVert}\right)$ :

| $G_{11}$ | $G_{12}$ | $G_{21}$ | $G_{22}$ |
|---|---|---|---|
| 86–87 % | **59–62 %** (le plus faible) | 96 % (échelon 10 V) | 82 % |

**Contrôles de cohérence.** Le même corps devrait donner les mêmes pôles :

- tangage : $\omega_n$ vaut 1,18 rad/s ($G_{11}$) contre 1,41 ($G_{12}$) ;
- lacet : $T$ vaut 1,9 s ($G_{22}$) contre 4,9 s ($G_{21}$).

La structure à 4 états est donc un **compromis**, dont la limite principale est $G_{12}$.

### 3.4 Validation du modèle identifié

- **En boucle ouverte, sur les essais 2 DDL (0–88 s) :**
  - modèle identifié : fit tangage ≈ 31 %, lacet ≈ 25 % ;
  - modèle Quanser : −60 % et −17 %, c'est-à-dire pire qu'une constante.
  
  Les fits sont bas pour les deux, parce que l'essai 2 DDL fait tourner le lacet vite (centrifuge, V²). Mais l'écart entre les deux modèles est net.
- **En boucle fermée** (`tune_lqr_model_refined.m`, essais LQR en rampe `fixed3_sys1..4`) : on simule le **même K** sur les deux modèles et on compare à la mesure.
  - Fits des angles : **84–95 % pour les deux modèles**. La boucle fermée **cache les erreurs de modèle**, car une consigne lente est suivie par n'importe quel modèle à peu près juste.
  - Fit de **l'erreur de suivi** : meilleur avec le modèle identifié pour 3 réglages sur 4 (+3 à +10 points).
  - **Erreur statique de tangage prédite :** le modèle identifié colle à la mesure **à 0,2° près** (3,0/2,7 ; 5,5/5,5 ; 2,9/2,8 ; 3,6/3,5°). Le modèle Quanser donne environ 40 % de moins. **C'est l'argument le plus fort de la validation.**
  - Dépassement du lacet surestimé (8 % contre 2 %) : le modèle linéaire n'a pas de frottement sec.
  - Pôle le plus lent de la boucle : −3,0 rad/s sur le modèle Quanser (conception), environ −0,8 rad/s sur le modèle identifié. La boucle réelle est plus lente que prévu.
- **Pourquoi le modèle Quanser « surestime » le moteur :** Quanser a identifié avec des **échelons de 20 V**. Or la poussée croît en V², donc à nos tensions (6–10 V) le couple par volt est plus faible.

### 3.5 Le frottement sec (Coulomb + Karnopp)

Script : `fit_friction_AERO.m`.

**Constat.** Les paramètres « linéaires » dépendaient de l'essai :

- le $\zeta$ du tangage valait 0,10 à 10 V et 0,16–0,21 à 8 V ;
- la constante de temps du lacet valait 2 s, 4,9 s ou 14 s selon l'essai.

Un vrai système linéaire n'a **qu'un** jeu de paramètres. Ici, ils dépendent de l'amplitude du mouvement, ce qui est la **signature du frottement sec**.

**Modèle.** On ajoute un couple constant, toujours opposé au mouvement, aux équations de Quanser :

$$J_p\ddot\theta + D_p\dot\theta + F_{c,p}\,\mathrm{sign}(\dot\theta) + K_{sp}\theta = K_{pp}V_p \qquad J_y\ddot\psi + D_y\dot\psi + F_{c,y}\,\mathrm{sign}(\dot\psi) = K_{yy}V_y$$

On y ajoute le **frottement statique** (modèle de Karnopp) : à l'arrêt, l'axe reste immobile tant que les autres couples restent plus petits que $F_c$.

**Comment $F_c$ a été calculé** (question probable) :

1. On simule le modèle non linéaire avec la tension mesurée (Euler explicite, avec la règle de Karnopp).
2. On minimise la somme des carrés des écarts mesure − simulation avec `lsqnonlin`, borne inférieure 0.
3. On ré-estime $D$ **en même temps** que $F_c$, sinon le frottement serait compté deux fois (une fois dans $D$, une fois dans $F_c$).
4. Pour le lacet, l'erreur porte sur la **vitesse** (qui montre les arrêts) et sur l'**angle** (qui montre la dérive), avec un poids de 0,3 sur l'angle.

CONTSID ne peut pas faire cette estimation, car le modèle est non linéaire.

**Explication du lien avec l'amortissement.** Un modèle linéaire ne peut imiter le frottement sec que par un amortissement visqueux équivalent (fonction descriptive) :

$$D_{eq} = \frac{4F_c}{\pi\,\omega A}$$

Plus l'amplitude $A$ est petite, plus l'amortissement apparent est grand. C'est exactement ce qu'on observait.

**Résultats.**

- **Fits :**
  - tangage : 85,5 → 87,5 % (estimation) ; validation sur la rampe de 1 V : 46 → 60 % ;
  - lacet : vitesse 74 → **87 %**, angle 82 → 85 %.
- **Vérifications « à la main » :**

  | Grandeur | Formule | Prédit | Mesuré |
  |---|---|---|---|
  | Bande de blocage du tangage | $F_{c,p}/K_{sp}$ | ±1,0° | +1,4 / −1,8° |
  | Tension de décollage du lacet | $F_{c,y}/K_{yy}$ | **5,8 V** | zone morte 6–7 V |
  | Arrêt du lacet depuis 18 °/s | — | ≈ 1,6 s | 1,7–1,8 s |

- La majeure partie de l'amortissement « linéaire » du tangage est en réalité du frottement : $\zeta$ visqueux passe de 0,16 à **0,05**.
- **Limite :** la validation du lacet sur `step_yaw_10V` est mauvaise (fit vitesse −43 %). Le modèle plafonne à 45 °/s alors que le lacet réel monte à environ 105 °/s, à cause de la poussée en V² (voir § 4).

**Paliers plats mais décalés d'environ 1,5° sur le tangage.** Deux causes s'ajoutent :

1. Le moteur de tangage est **dissymétrique** : −8 V donne −12 à −13°, alors que +8 V ne donne que +9 à +10°. Un seul $K_{pp}$ se place sur la moyenne.
2. Le frottement bloque l'axe **n'importe où** dans la bande de ±1°.

**Quel jeu utiliser ?**

- Pour la **synthèse** (LQR, PD, découpleur) : le jeu **linéaire CONTSID**. C'est la meilleure approximation linéaire, avec le frottement inclus « en moyenne ».
- Le jeu « frottement » ($D_p$, $D_y$, $K_{yy}$, $F_c$) sert à la **simulation** et à l'explication. Ses valeurs vont **ensemble** : ne jamais les utiliser sans $F_c$.

### 3.6 Le test de linéarité demandé par l'enseignant : impulsions 0 → X V

**Le test.** On applique des impulsions sur le moteur de lacet seulement ($V_p = 0$, les deux axes libres). Chaque niveau dure 5 s, suivi de 5 s à 0 V, puis on passe au niveau suivant.

- `custom_yaw_0to20` : +1, +2, … +20 V. L'AERO s'est **déconnecté pendant le palier à 17 V** : les codeurs retombent à 0. Les données sont coupées au premier saut de plus de 10° en un échantillon, ce qui garde les niveaux **1 à 16 V**.
- `custom_yaw_0to12-12` : +1, −1, +2, −2, … +12, −12 V. Ce fichier sert à tester les **deux signes**.

**Le principe : l'enseignant a raison.** Vérifier que K et τ sont les mêmes pour chaque 0 → X V, c'est exactement le **test d'homogénéité** (proportionnalité), la première propriété d'un système linéaire :

$$u \;\rightarrow\; y \quad\Longrightarrow\quad \alpha u \;\rightarrow\; \alpha y$$

Si le lacet était linéaire, on observerait les quatre choses suivantes :

- la réponse à X volts serait **X fois** la réponse à 1 V ;
- les courbes $\dot\psi(t)/V$ se superposeraient toutes (**une seule courbe**) ;
- $K = \dot\psi(5\,\text{s})/V$ et $\tau_{63\%}$ seraient **constants** ;
- la vitesse atteinte serait une **droite passant par 0** en fonction de $V$.

La 2ᵉ propriété, l'**additivité** (superposition), a déjà été testée en boucle ouverte avec `step_both_10V10V` comparé à la somme des deux échelons seuls. Elle n'est vraie que pendant les 2 premières secondes : ensuite, on mesure θ final = 31° contre 17° pour la somme.

**Résultats** (vitesse moyenne sur la dernière demi-seconde de chaque impulsion) :

| $V_y$ | Vitesse après 5 s (°/s) | K (°/s/V) | $\tau_{63}$ (s) | Hélice $\omega/V$ (rad/s/V) |
|---|---|---|---|---|
| 1 à 6 V | **0** (zone morte) | 0 | — | 12 (à 1 V) puis 20–21 |
| 7 V | 1,5 (+7 V : 5,4 / −7 V : −4,0) | 0,2–0,8 | — | 19,2 |
| 8 V | 5–8 (−8 V : −11,8) | 0,65–1,0 (−8 V : 1,48) | 0,7–1,4 | 18,9 |
| 9 V | 21–23 (−9 V : −29) | 2,3–2,5 (−9 V : 3,2) | 1,5–2,4 | 18,6 |
| 10 V | 29–36 (−10 V : −46) | 2,9–3,6 (−10 V : 4,6) | 1,9–2,8 | 18,4 |
| 11 V | 47–50 (−11 V : −57) | 4,2–4,6 (−11 V : 5,2) | 2,0–2,8 | 18,1 |
| 12 V | 54–59 (−12 V : ≈ −69) | 4,5–4,9 (−12 V : ≈ 5,7) | 2,5–2,9 | 17,9 |
| 13 V | 74 | 5,7 | 2,4 | 17,6 |
| 14 V | 86 | 6,1 | 2,1* | 17,4 |
| 15 V | 120 | 8,0 | 2,0* | 17,2 |
| 16 V | 141 | **8,8** | 1,2* | 16,9 |
| **Modèle linéaire** | $1{,}63\,V$ | **1,63 constant** | **≈ 1,7 constant** | — |

\* À 14–16 V, le lacet tournait encore au début de l'impulsion (3,5 ; 11 ; 32 °/s), parce que 5 s de repos ne suffisaient plus à l'arrêter. Ces τ sont donc peu fiables : ce sont les marqueurs blancs du script.

Le modèle linéaire CONTSID donne $K_{22} = K_{yy}/D_y = 1{,}76$ °/s/V en régime établi, soit $1{,}763\,(1-e^{-5/1{,}91}) = 1{,}63$ °/s/V au bout de 5 s.

**Lecture des résultats :**

1. **Zone morte :** rien ne bouge jusqu'à 6 V, pour les deux signes. Le décollage se fait entre 6 et 7 V, ce qui est cohérent avec $F_{c,y}/K_{yy} = 5{,}8$ V.
2. **Gain non constant :** K passe de 0 à **8,8 °/s/V**. Le modèle linéaire n'est juste qu'autour de **8–9 V**, c'est-à-dire là où il a été identifié (créneaux ±8 V). À 16 V, il prédit 26 °/s pour 141 °/s mesurés.
3. **Caractéristique statique en V² :** $\dot\psi(5\,\text{s}) \approx 0{,}68\,(V^2 - 50)$ °/s, avec $V_0 \approx 7{,}1$ V. Un ajustement affine $17\,(V - 7{,}8)$ colle moins bien.
4. **Tension négative plus efficace :** de 20 à 30 % de plus que la tension positive (dissymétrie moteur/hélice).
5. **τ variable** (0,7 à 2,9 s). Attention : avec du frottement sec, le τ apparent change avec V même si $D_y$ est constant. **τ(V) seul ne prouve donc pas que $D_y$ change.** C'est K(V) qui est la preuve nette.
6. **Le moteur lui-même est presque linéaire.** La vitesse d'hélice par volt reste entre 20 et 17 rad/s/V de 2 à 16 V (−15 %), et elle est symétrique à 1 % près. Seul 1 V fait exception (12 rad/s/V : frottement du moteur). **La non-linéarité est donc en aval du moteur** :

$$V_y \xrightarrow{\ \text{moteur : quasi linéaire}\ } \omega \xrightarrow{\ \text{poussée} \propto \omega^2\ } \tau \xrightarrow{\ \text{frottement sec}\ } \dot\psi$$

7. **Freinage à $V = 0$ :** la décélération est à peu près **constante**, 13–16 °/s² de 8 à 14 V. C'est la signature du frottement de Coulomb : un couple constant donne une décélération constante, donc un temps d'arrêt proportionnel à la vitesse. Ordre de grandeur théorique : $F_{c,y}/J_y = 0{,}19$ rad/s² ≈ 11 °/s². Au-delà de 15 V, la décélération augmente (traînée aérodynamique à grande vitesse).
8. **Le modèle avec frottement** reproduit bien la zone morte, mais pas la croissance en V². À 16 V, il prédit environ 63 °/s pour 141 °/s mesurés.
9. **Couplage :** le tangage est poussé par $V_y$ ($G_{12}$, couple de réaction). Pendant l'impulsion de −12 V, le tangage descend jusqu'à environ −32°.

**Conclusion à dire :**

> « Le test d'homogénéité montre que le lacet est fortement non linéaire : zone morte jusqu'à 6 V, puis un gain par volt multiplié par 9 entre 8 et 16 V, avec une caractéristique en V². Le moteur seul est presque linéaire ; la non-linéarité vient de la poussée de l'hélice et du frottement sec. Un modèle linéaire n'est donc valable qu'autour de son point d'identification (8 V), qui correspond aux tensions utilisées par nos correcteurs avec des consignes en rampe. »

**Pistes de ré-estimation** (perspectives, pas faites, à citer en conclusion) :

- **Modèles linéaires locaux :** un `procsrivc('P1I')` par niveau, ce qui donne une table K(V), T(V) pour un **séquencement de gain**.
- **Modèle de Hammerstein :** non-linéarité statique $u' = f(V)$ (zone morte + $V\lvert V\rvert$), suivie du P1I linéaire. Si K et τ deviennent constants avec $u'$ comme entrée, la structure est validée.
- **Linéarisation de l'entrée** dans la commande : $V = \mathrm{sign}(u)\sqrt{\lvert u\rvert}$, avec une compensation de frottement $\approx V_0\,\mathrm{sign}(u)$.
- Faire le **même test sur le tangage**, ce qui est plus simple grâce au ressort : l'angle final donne directement la courbe de poussée. Limiter les essais à 15 V.

---

## 4. Synthèse : linéaire ou non linéaire ? (tableau pour l'oral)

| Propriété / phénomène | Test réalisé | Ce qu'on observe | Conséquence |
|---|---|---|---|
| **Homogénéité** | échelons 5/10/15 V (réponses divisées par V) ; impulsions 0 → X V | pic de tangage / V² ≈ constant (0,26 ; 0,255 ; 0,22) ; K du lacet de 0 à 8,8 °/s/V | ❌ non linéaire |
| **Additivité** | 10 V + 10 V comparé à la somme des deux échelons | juste pendant ≈ 2 s, puis θ = 31° contre 17° | ❌ linéaire **localement** seulement |
| **Poussée ∝ V²** | escalier de tangage, impulsions lacet | $\theta \approx 0{,}140\,V\lvert V\rvert$ (V>0), $0{,}187\,V\lvert V\rvert$ (V<0) ; $\dot\psi \approx 0{,}68(V^2-50)$ | gain qui dépend du point de fonctionnement ; avec $u' = V\lvert V\rvert/8$, fit 70 → 85 % |
| **Frottement sec** | paliers, arrêt du lacet, zone morte | zone morte 6–7 V, bande ±1° au tangage, décélération constante | erreur statique, blocage du lacet (stick-slip) |
| **Dissymétrie** | créneaux ±8 V, impulsions ±V | tangage : −V donne 25–33 % de plus ; lacet : −V donne 20–30 % de plus | décalage des paliers d'environ 1,5° |
| **Effet centrifuge** | essais 2 DDL, échelons à 10–15 V | le tangage retombe quand le lacet tourne vite ; terme $\dot\psi^2\sin\theta\cos\theta$ | fit 2 DDL du tangage 50 → 70 % ; négligeable en rampe (20–40 °/s) |
| **Couples de réaction** | $G_{12}$, $G_{21}$ | « coup » pendant la montée en vitesse de l'hélice | $G_{12}$ mal décrit (60 %) : il manque un zéro |
| **Retard** | `tfsrivc` | 0,05–0,12 s | absent du modèle à 4 états |

**Domaine de validité du modèle linéaire identifié :** tensions autour de 8 V et lacet lent. C'est justement le domaine des correcteurs avec consignes en rampe (0 % de saturation, lacet à 20–40 °/s).

---

## 5. Tableau final des paramètres

| Paramètre | Quanser | Linéaire (CONTSID) | Avec frottement |
|---|---|---|---|
| $J_p$ (kg·m²) | 0,02189 | 0,02189 (conservé) | 0,02189 |
| $J_y$ (kg·m²) | 0,02199 | 0,02199 (conservé) | 0,02199 |
| $K_{sp}$ (N·m/rad) | 0,03746 | 0,03069 | 0,03151 |
| $D_p$ (N·m·s/rad) | 0,00711 | 0,00845 | 0,00273 |
| $\zeta$ tangage | 0,124 | 0,163 | 0,052 |
| $\omega_n$ tangage (rad/s) | 1,31 | 1,18 | 1,20 |
| $K_{pp}$ (N·m/V) | 0,00110 | 0,000738 | 0,000744 |
| $K_{py}$ (N·m/V) | 0,00210 | ≈ 0,00061 | idem linéaire |
| $D_y$ (N·m·s/rad) | 0,0220 | 0,01154 | 0,00372 |
| $J_y/D_y$ (s) | 1,0 | 1,91 | 5,92 |
| $K_{yy}$ (N·m/V) | 0,0022 | 0,000355 | 0,000710 |
| $K_{yp}$ (N·m/V) | −0,0027 | −0,00065 | idem linéaire |
| $F_{c,p}$ (N·m) | — | — | 0,00057 (bande ±1,0°) |
| $F_{c,y}$ (N·m) | — | — | 0,00415 (décollage 5,8 V) |

> Vérifiez $K_{py}$ dans `quanser_aero_parameters_identified.mat` avant de l'écrire sur une diapo (≈ 0,00061 avec l'essai libre, 0,00066 avec l'essai bloqué).

Ce qu'il faut retenir du tableau :

- Le moteur réel donne **1,5 à 6 fois moins de couple par volt** que le modèle Quanser.
- Quanser annonce $K_{py} \approx 1{,}9\,K_{pp}$, alors qu'on identifie $K_{py} \approx 0{,}8\text{–}0{,}9\,K_{pp}$.

---

## 6. Remarques sur les scripts Quanser (si la question vient)

- `find_damping_pitch.m` utilise $\zeta = 1/\sqrt{1 + 2\pi^2/\sigma}$ au lieu de l'éq. 2.13 du guide. Elle donne 0,124 au lieu de 0,05–0,07.
- `find_thrust_pitch_from_yaw.m` utilise $D_y$ là où on attendrait $D_p$.

À présenter comme « nous avons remarqué », sans accuser : cela explique en partie pourquoi on a ré-identifié plutôt que repris leurs valeurs.

---

## 7. Tri des fichiers `.m`

**Légende des statuts :**

- **PRÉSENTATION** : on en tire une figure pour demain.
- **ANNEXE** : utile pour les questions.
- **OUTIL** : fonction ou script appelé par d'autres.
- **ARCHIVE** : remplacé par une version plus récente.

### 7.1 Modèle fourni par Quanser

| Fichier | Rôle | Statut |
|---|---|---|
| `quanser_aero_parameters.m` | Paramètres Quanser ($J_p$, $J_y$, $K_{sp}$…) | OUTIL |
| `quanser_aero_state_space.m` | A, B, C, D (le `clear` a été retiré) | OUTIL |
| `quanser_aero_lqr.m` | LQR d'origine Quanser | OUTIL |

### 7.2 Essais en boucle ouverte (non-linéarités vues aux échelons)

| Fichier | Rôle | Statut |
|---|---|---|
| `step_response_AERO.m` / `step_response_AERO_impr.m` | Trace **un** essai à 12 lignes | ANNEXE |
| `plot_all_steps_AERO.m` (dans le projet sous le nom **`step_input_comparasion.m`**, même contenu) | 7 échelons : fig. 3 = réponses / V (**homogénéité**), fig. 4 = **superposition** | **PRÉSENTATION** (diapo 4) |
| `plot_all_tests_AERO.m` | Tous les essais `custom_*` dans une fenêtre à onglets (en français) + onglet Résumé | ANNEXE |

### 7.3 Identification du modèle

| Fichier | Rôle | Statut |
|---|---|---|
| `identify_steps_AERO.m` | 1ʳᵉ tentative : CONTSID sur les échelons, ordres libres | ARCHIVE |
| `identify_quanser_AERO.m` | **Identification finale** (structure Quanser, `tfsrivc` + `procsrivc P1I`), validation 2 DDL, enregistre `quanser_aero_parameters_identified.mat` | **PRÉSENTATION** (diapos 9–10) |
| `fit_friction_AERO.m` | Frottement sec (Karnopp, `lsqnonlin`), enregistre `aero_friction_parameters.mat` | **PRÉSENTATION** (diapo 11) |

### 7.4 Test de linéarité (impulsions 0 → X V)

| Fichier | Rôle | Statut |
|---|---|---|
| `plot_yaw_linearity_AERO.m` (racine et `claude/`) | 1ʳᵉ version (3 figures) | ARCHIVE |
| `claude/plot_yaw_linearity_report_AERO.m` | **Version rapport** : ajoute la vitesse d'hélice (moteur seul), le modèle linéaire passé dans **la même** impulsion de 1 V, les marqueurs blancs « lacet encore en mouvement » et l'export PNG | **PRÉSENTATION** (diapo 11) |

### 7.5 LQR

| Fichier | Rôle | Statut |
|---|---|---|
| `tune_lqr_AERO.m` | Simulation des 4 réglages sur le modèle Quanser, comparaison à la mesure | PRÉSENTATION (diapo 7, autre orateur) |
| `plot_lqr_AERO.m` / `plot_lqr_AERO_impr.m` | Analyse d'**un** essai LQR (9 lignes) | PRÉSENTATION (diapo 7) |
| `compare_lqr_ramp_AERO.m` | Compare les 4 essais en rampe | PRÉSENTATION (diapo 8) |
| `computeMetrics.m`, `modelError.m` | Indicateurs communs à tous les correcteurs | OUTIL |
| `tune_lqr_model_refined.m` | **Même K simulé sur le modèle Quanser ET sur le modèle identifié**, comparé à la mesure (onglets + onglet « Comparison » + nouveau K) | **PRÉSENTATION** (diapo 10) |

> Défaut connu de `tune_lqr_model_refined.m` : l'indicateur « tangage déplacé par le lacet » vaut toujours 0, car la consigne de tangage n'est jamais constante pendant tout un mouvement de lacet. Ne montrez pas cette barre.

### 7.6 PD découplé (autre conversation, non revu ici en détail)

| Fichier | Rôle | Statut |
|---|---|---|
| `decoupled_pd_AERO.m` | Synthèse + simulation du PD découplé | ANNEXE |
| `setup_decoupled_pd_simulink.m` | Gains pour le modèle Simulink (blocs PIV) | OUTIL |
| `decoupled_pd_volt_torque.m` | Variante tension / couple | ANNEXE |
| `plot_decoupled_pd_tests_AERO.m` | Analyse des essais `aero_dpd_rsp_test1..3` | ANNEXE |
| `claude/figures_pd_presentation_AERO.m` | **4 figures PNG en français** (suivi, même rampe LQR vs PD, indicateurs, couple obtenu / demandé) | **PRÉSENTATION** (diapo 13) |

> **Doublons :** plusieurs fichiers existent à la racine du projet ET dans `claude/` (`decoupled_pd_AERO.m`, `plot_decoupled_pd_tests_AERO.m`, `setup_decoupled_pd_simulink.m`, `plot_yaw_linearity_AERO.m`). Les copies à la racine ont été déposées le 15/10 : ce sont a priori celles que vous avez lancées. Gardez-les et ignorez les copies `claude/`, **sauf** `plot_yaw_linearity_report_AERO.m` et `figures_pd_presentation_AERO.m`, qui n'existent que dans `claude/`.

---

## 8. Quelles figures montrer demain (axe linéarité / non-linéarité)

**Fichiers nécessaires dans le dossier MATLAB :** les `.mat` des essais, `quanser_aero_parameters_identified.mat`, `aero_friction_parameters.mat` et CONTSID dans le path (pour `identify_quanser_AERO.m`).

### 8.1 Le jeu minimal : 6 figures, dans l'ordre de l'histoire

| # | Diapo | Script → figure | Message à dire | Linéarité |
|---|---|---|---|---|
| 1 | 4 | `plot_all_steps_AERO.m` → **fig. 3** « réponses divisées par la tension » | « Si le système était linéaire, les courbes 5/10/15 V se superposeraient : elles ne le font pas. » | homogénéité ❌ |
| 2 | 4 (ou annexe) | `plot_all_steps_AERO.m` → **fig. 4** « superposition » | « La somme des réponses n'est juste que 2 s : linéaire seulement localement. » | additivité ❌ |
| 3 | 9 | `identify_quanser_AERO.m` → figure « Identification of the 4 transfer functions » (garder G11 et G22 si la place manque) | « Structure Quanser, paramètres CONTSID : 86 % et 82 %. » | modèle linéaire local |
| 4 | 10 | `tune_lqr_model_refined.m` (série `fixed3`) → onglet **Comparison**, barre « Pitch steady-state error » | « Le modèle identifié prédit l'erreur réelle à 0,2° près ; Quanser 40 % de moins. » | validation |
| 5 | 11 | `fit_friction_AERO.m` → figure **« Yaw: linear vs friction model »** (estimation : angle + vitesse) | « Avec Fc·sign(ψ̇), le modèle s'arrête comme le vrai lacet : fit vitesse 74 → 87 %. » | frottement sec |
| 6 | 11 | `plot_yaw_linearity_report_AERO.m`, `fileName = 'custom_yaw_0to20'` → **fig. 3** « indicators vs V » | « Le test de l'enseignant : K devrait être constant, il passe de 0 à 8,8 °/s/V ; le moteur seul (c) est presque linéaire ; caractéristique (d) en V² avec zone morte. » | homogénéité ❌, V², zone morte |

### 8.2 Figures de réserve (annexes, en cas de questions)

| Script → figure | Utile pour la question… |
|---|---|
| `plot_yaw_linearity_report_AERO.m` (`custom_yaw_0to20`) → fig. 2 (vitesse / V, une seule courbe si linéaire) | « Montrez-nous les réponses brutes du test d'homogénéité. » |
| `plot_yaw_linearity_report_AERO.m` (`custom_yaw_0to12-12`) → fig. 3 (d) | « Et en tension négative ? » (zone morte symétrique, −V 20–30 % plus fort) |
| `plot_yaw_linearity_report_AERO.m` → fig. 1 (vue d'ensemble) | « Comment s'est déroulé l'essai ? » (+ déconnexion à 17 V) |
| `identify_quanser_AERO.m` → figure « Validation custom_free_2dof1 » | « Avez-vous validé sur un autre essai ? » (identifié 31/25 % contre Quanser −60/−17 %) |
| `fit_friction_AERO.m` → figure « Pitch: linear vs friction model » | « Et le frottement du tangage ? » (paliers plats, bande ±1°, dissymétrie) |
| `tune_lqr_model_refined.m` → onglet d'un réglage (suivi Quanser / identifié / mesuré) | « La boucle fermée cache-t-elle les erreurs de modèle ? » |
| `plot_all_tests_AERO.m` → onglets | « Quels essais avez-vous faits ? » |

### 8.3 Mise en forme rapide

- **Enregistrer une figure :** `exportgraphics(gcf, 'nom.png', 'Resolution', 300)`. `plot_yaw_linearity_report_AERO.m` le fait déjà (`savePng = true`) et crée `custom_yaw_0to20_overview.png`, `_per_volt.png` et `_indicators.png`.
- **Figures à onglets** (`tune_lqr_model_refined.m`, `plot_all_tests_AERO.m`) : affichez l'onglet voulu, puis faites une capture d'écran (Win + Maj + S).
- **Police :** ajoutez `set(findall(gcf,'Type','axes'),'FontSize',14)` avant l'export, pour que la figure soit lisible de loin.
- **Titres en anglais** (scripts d'identification et de linéarité) : traduisez au moins le titre de la figure sur la diapo, ou mettez le titre en français dans la diapo elle-même. Correspondances utiles : *yaw rate* = vitesse de lacet, *pitch* = tangage, *gain per volt* = gain par volt, *dead zone* = zone morte, *propeller speed* = vitesse d'hélice, *measured / model* = mesure / modèle.

---

## 9. Questions probables sur le modèle, avec réponses courtes

1. **« Le test K et τ identiques pour chaque échelon, est-ce le bon test de linéarité ? »**
   Oui, c'est le test d'homogénéité. Avec le test d'additivité (superposition, fait aux échelons), ce sont les deux propriétés qui définissent un système linéaire. Les deux échouent ici.
2. **« Pourquoi τ change-t-il ? $D_y$ n'est donc pas constant ? »**
   Pas forcément. Avec du frottement de Coulomb, la réponse n'est plus une exponentielle, et le τ « à 63 % » dépend de l'amplitude même si $D_y$ est constant. La preuve nette de non-linéarité est K(V), et surtout la zone morte.
3. **« Si le système est si non linéaire, pourquoi un modèle linéaire ? »**
   Le LQR et le PD se calculent sur un modèle linéaire. On l'a identifié **au point de fonctionnement des correcteurs** (8 V, lacet lent, consignes en rampe), et il y prédit l'erreur réelle à 0,2° près.
4. **« Pourquoi le modèle Quanser est-il moins bon ? »**
   Il a été identifié avec des échelons de 20 V. Comme la poussée croît en V², le couple par volt est 1,5 à 6 fois plus faible à nos tensions.
5. **« Comment avez-vous obtenu $F_c$ ? Où intervient-il ? »**
   C'est un terme $F_c\,\mathrm{sign}(\dot q)$ ajouté au membre de gauche des équations de Quanser, avec la règle de Karnopp à l'arrêt. Il est estimé par moindres carrés non linéaires (`lsqnonlin`) sur la simulation complète, en même temps que $D$. On l'a vérifié à la main : $F_{c,y}/K_{yy} = 5{,}8$ V pour une zone morte mesurée de 6–7 V.
6. **« Pourquoi ne pas utiliser le modèle avec frottement pour la commande ? »**
   $\mathrm{sign}(\dot q)$ n'est pas linéarisable en 0. Il sert à la simulation et justifie une action intégrale ou une compensation de frottement.
7. **« Pourquoi $G_{12}$ est-il à 60 % ? »**
   Le couplage vient du couple de réaction de l'hélice pendant sa mise en vitesse. Il faudrait un zéro, que la structure Quanser n'a pas. Par ailleurs, $G_{12}$ partage le dénominateur de $G_{11}$.
8. **« Pourquoi `procsrivc` pour le lacet et `tfsrivc` pour le tangage ? »**
   `procsrivc` ne fait que des 1ᵉʳ ordres (`P1D`, `P1I`). P1I correspond exactement au lacet (intégrateur + 1ᵉʳ ordre). Le tangage est un 2ᵉ ordre, donc il faut `tfsrivc(data, 2, 0)`.
9. **« La boucle fermée valide-t-elle le modèle ? »**
   Pas les fits d'angle : 84–95 % pour les deux modèles, car la rétroaction masque les erreurs. Ce qui le valide, c'est **l'erreur statique** et le **fit de l'erreur de suivi**, qui sont sensibles au gain du modèle.
10. **« Pourquoi l'essai s'arrête-t-il à 16 V ? »**
    L'AERO s'est déconnecté pendant le palier à 17 V (saut des codeurs). Pour de futurs essais : rester ≤ 15 V et allonger le repos, car à 14–16 V, 5 s ne suffisent pas pour que le lacet s'arrête.
11. **« Que feriez-vous avec plus de temps ? »**
    Un modèle de Hammerstein (non-linéarité statique + P1I), un séquencement de gain K(V), une linéarisation de l'entrée $V = \mathrm{sign}(u)\sqrt{\lvert u\rvert}$, une compensation du frottement, une action intégrale (LQI), et des essais répétés.

---

## 10. Formules prêtes à coller (PowerPoint → Équation → LaTeX)

**Homogénéité :**
```
u(t) \rightarrow y(t) \;\Rightarrow\; \alpha\,u(t) \rightarrow \alpha\,y(t)
```

**Additivité :**
```
u_1 + u_2 \rightarrow y_1 + y_2
```

**Fit :**
```
\text{fit} = 100\left(1-\frac{\lVert y-\hat y\rVert}{\lVert y-\bar y\rVert}\right)
```

**$G_{22}$ (P1I) :**
```
G_{22}(s)=\frac{K_{yy}}{s\,(J_y s + D_y)}=\frac{K}{s\,(1+Ts)},\quad K=\frac{K_{yy}}{D_y},\ T=\frac{J_y}{D_y}
```

**Lacet avec frottement :**
```
J_y\ddot\psi + D_y\dot\psi + F_{c,y}\,\mathrm{sign}(\dot\psi) = K_{yy}V_y
```

**Fonction descriptive :**
```
D_{eq} = \frac{4F_c}{\pi\,\omega A}
```

**Tension de décollage :**
```
V_0 = \frac{F_{c,y}}{K_{yy}} \approx 5{,}8\ \text{V}
```

**Caractéristique statique :**
```
\dot\psi_{5\,s} \approx 0{,}68\,(V_y^2 - 50)\ ^\circ/\text{s}
```

**Poussée :**
```
F = k_f\,\omega^2,\quad \omega \propto V \;\Rightarrow\; \tau \propto V\lvert V\rvert
```

**Chaîne :**
```
V_y \xrightarrow{\text{moteur (quasi linéaire)}} \omega \xrightarrow{\ \propto\,\omega^2\ } \tau \xrightarrow{\text{frottement sec}} \dot\psi
```

---

## 11. Paragraphe de synthèse (rapport ou conclusion orale)

Nous avons conservé la structure physique du modèle de Quanser et ré-identifié ses paramètres avec CONTSID : `tfsrivc` pour le tangage (2ᵉ ordre) et `procsrivc('P1I')` pour le lacet (1ᵉʳ ordre + intégrateur), à partir de créneaux de ±8 V. Le modèle identifié prédit l'erreur statique réelle du LQR à 0,2° près, là où le modèle de Quanser, identifié à 20 V, la sous-estime de 40 %.

Les essais montrent cependant que l'AERO n'est linéaire que localement. Le principe de superposition n'est vérifié que pendant les deux premières secondes. Le test d'homogénéité sur le lacet (impulsions de 0 à 16 V) donne une zone morte jusqu'à 6 V, puis un gain par volt qui passe de 1 à 8,8 °/s/V, avec une caractéristique en V².

Le moteur lui-même est quasi linéaire. Les non-linéarités viennent de la poussée des hélices, proportionnelle au carré de leur vitesse, et du frottement sec, que nous avons modélisé par un terme de Coulomb ($F_{c,y}/K_{yy} = 5{,}8$ V, cohérent avec la zone morte mesurée). Le modèle linéaire reste donc valable dans le domaine de fonctionnement de nos correcteurs (tensions proches de 8 V, consignes en rampe), et ses limites justifient les perspectives : action intégrale, compensation du frottement et linéarisation de la poussée.

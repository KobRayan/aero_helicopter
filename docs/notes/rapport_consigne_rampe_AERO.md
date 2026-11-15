# Consigne en rampe et comparaison de 4 LQR sur le Quanser AERO

*Mini-projet AERO – Polytech Nancy – 06/10/2026*

## 1. Objectif

Les essais LQR précédents utilisaient des consignes en échelon. L'enseignant a proposé quatre pistes :

1. remplacer l'échelon par une **rampe**, pour éviter le passage brusque ;
2. **limiter la consigne** pour rester dans le domaine linéaire ;
3. un PID découplé, plus tard ;
4. un **gain de pré-contrôle**.

Ce rapport traite des pistes 1 et 2. Il compare les 4 réglages LQR, en simulation et sur l'AERO, et conclut sur la piste 4 et les étapes suivantes.

## 2. Mise en œuvre de la consigne en rampe

### 2.1 Pourquoi une rampe

La commande LQR vaut u = K (x_ref − x). Avec un échelon de 40° en tangage, l'erreur de 0,70 rad multipliée par K₁₁ ≈ 98 V/rad demande environ 68 V : les moteurs saturent à ±20 V. Pendant la saturation, la boucle n'est plus linéaire. Le terme croisé K₁₂ ≈ −103 V/rad envoie aussi une grande tension sur le moteur de tangage lors d'un échelon de lacet, d'où un fort couplage.

Avec une rampe, l'erreur reste de quelques degrés : la commande reste proportionnée et sous la saturation.

Une simulation préliminaire (modèle Quanser, LQR « Current », saturation ±20 V) a guidé le choix de la pente :

| Consigne | \|Vp\| max | \|Vy\| max | Temps saturé | Erreur max de tangage |
|---|---|---|---|---|
| Échelon | 20 V | 20 V | 9,6 % | 38° |
| Rampe 10 °/s | 5,9 V | 6,4 V | 0 % | 4,7° |
| **Rampe 20 °/s** | 8,4 V | 10,4 V | 0 % | 7,5° |
| Rampe 40 °/s | 16 V | 18,7 V | 0 % | 13° |

**20 °/s** est un bon compromis : une variation de 40° dure 2 s, ce qui tient dans les demi-périodes de 5 s (tangage) et 12,5 s (lacet), et la tension reste à environ la moitié de la limite.

### 2.2 Réalisation dans Simulink

- **Première version :** six échelons sommés, un intégrateur et un gain de 10. Elle donnait bien des rampes de 20 °/s, mais le motif était codé à la main (une impulsion manquait en tangage) et le premier segment, jusqu'à −20°, était absent. Elle a été abandonnée.
- **Version retenue :** un bloc **Rate Limiter** (pentes +20 / −20 °/s), précédé d'un bloc **Saturation** qui borne la consigne (±20° en tangage, ±40° en lacet).
- **Problème à t = 0 :** avec un solveur à pas variable, le Rate Limiter recopie son entrée au premier pas sans appliquer sa condition initiale, donc la consigne démarrait directement à −20°. Solution robuste : multiplier la consigne par un échelon 0 → 1 à t = 1 s (blocs Step et Product). La consigne vaut 0 pendant 1 s, puis rejoint −20° par une rampe.
- **Pourquoi partir de 0 :** les codeurs sont relatifs et valent 0 au lancement. Une consigne de −20° dès t = 0 recréerait un saut d'erreur. Il faut donc tenir le bras à l'horizontale au lancement, pour que le zéro des codeurs corresponde à l'équilibre du modèle linéarisé.

### 2.3 Adaptation des scripts

- **`computeMetrics.m` :** les « marches » étaient détectées par `diff(ref) ~= 0`. Avec une rampe, chaque échantillon de 2 ms devenait une nouvelle marche, ce qui donnait des temps de montée et d'établissement en NaN, un dépassement faux et un couplage faux. La nouvelle fonction `moveStarts` regroupe les échantillons consécutifs qui changent en un seul **mouvement**, dont la cible est le palier final. Les résultats sur des échelons sont inchangés.
- **`tune_lqr_AERO.m` :** `iUsed` est maintenant lu dans le nom du fichier (`..._sysN` → réglage N), et les repères de la figure 4 utilisent la même détection de mouvements.
- **Nouveau script `compare_lqr_ramp_AERO.m` :** il compare les 4 essais (mesure et simulation), affiche des tableaux et trace 6 figures.

## 3. Protocole

- **Consignes :** tangage ±20°, lacet ±40°, rampes de 20 °/s, 30 s, Ts = 2 ms, mêmes consignes pour les 4 essais (vérifié).
- **Unités vérifiées :** le plus petit pas mesuré vaut 0,1758° en tangage et 0,0879° en lacet, soit exactement un pas codeur.
- **Réglages,** avec x = [θ, ψ, θ̇, ψ̇] :

| Essai | Réglage | Q | R | K_vel max (V·s/rad) |
|---|---|---|---|---|
| sys1 | Current | diag(200, 75, 0, 0) | 0,005·I | 45,5 |
| sys2 | Softer | diag(200, 75, 0, 0) | 0,02·I | 30,3 |
| sys3 | More damping | diag(200, 75, 20, 10) | 0,005·I | 69,0 |
| sys4 | Bryson | 1/(5°)², 1/(10°)², 1/(60 °/s)² | **0,005·I** (≈ 14,1 V) | 42,0 |

Le réglage réellement utilisé a été vérifié en identifiant K à partir des logs, par moindres carrés de u sur (x_ref − x). Pour sys4, le K identifié correspond à **R = 0,005** et non à 1/15² = 0,0044 comme prévu. Les simulations de ce rapport utilisent donc R = 0,005.

## 4. Métriques

Les métriques sont calculées de la même façon pour la mesure et pour la simulation.

| Métrique | Définition | Remarque avec une rampe |
|---|---|---|
| RMSE | √moyenne((réf − angle)²) sur tout l'essai | comparable entre essais de même consigne |
| t_r, t_s | montée 10–90 %, établissement à ±5 % | **incluent la durée de la rampe** (2 s pour 40°) : ne mesurent plus la vitesse du correcteur |
| Dépassement | max au-delà de la cible, en % du mouvement | comparable |
| e_ss | \|réf − moyenne de la dernière seconde\| | comparable |
| RMS V, % saturé | tension efficace, temps à ±20 V | comparable |
| Activité de tension | Σ\|ΔV\| / T (V/s) | mesure le bruit de commande |
| Couplage | écart max d'un axe pendant un mouvement de l'autre seul | voir les limites (§ 6.6) |
| Fit | 100·(1 − ‖y_mes − y_sim‖ / ‖y_mes − moy‖) | 100 % = modèle parfait |

## 5. Résultats

Valeurs mesurées sur l'AERO, avec la simulation (modèle Quanser nominal) entre parenthèses. Les médianes sont prises sur les mouvements ; les valeurs sont données en tangage / lacet.

| | Current | Softer | More damping | Bryson (R = 0,005) |
|---|---|---|---|---|
| RMSE (°) | **3,88** / **5,30** (3,66 / 3,80) | 5,79 / 8,52 (5,17 / 5,43) | 4,94 / 6,86 (4,91 / 5,41) | 4,45 / 7,35 (4,15 / 5,03) |
| Dépassement (%) | 0 / 1,0 (0 / 1,3) | 0 / 0,7 (0 / 1,6) | 0 / 1,0 (0 / 0,4) | 0 / 1,1 (0 / 1,1) |
| e_ss tangage (°) | **2,98** (1,60) | 6,09 (3,21) | 3,17 (1,60) | 3,92 (1,98) |
| e_ss lacet (°) | 1,00 (0,57) | 1,54 (0,81) | **0,82** (0,40) | 1,55 (0,76) |
| RMS V (V) | 6,2 / 6,2 (3,1 / 3,0) | 5,7 / 5,8 (2,8 / 2,6) | 6,1 / 6,2 (2,9 / 2,8) | 5,9 / 6,2 (3,0 / 2,9) |
| Saturation (%) | 0 / 0 | 0 / 0 | 0 / 0 | 0 / 0 |
| Activité (V/s) | 46 / 61 (4,1 / 4,9) | **32** / **43** (2,5 / 3,0) | 70 / 90 (3,1 / 3,7) | 41 / 55 (3,3 / 4,1) |
| Couplage θ←ψ / ψ←θ (°) | **1,08** / 1,32 (0,70 / 0,95) | 1,83 / 1,48 (1,81 / 1,51) | 2,84 / **1,12** (2,28 / 0,74) | 1,94 / 1,12 (1,07 / 1,33) |
| Fit (%) | 92,4 / 94,5 | 83,8 / 89,3 | 92,1 / 94,9 | 90,5 / 91,7 |

## 6. Analyse

### 6.1 Ce que la rampe a apporté

En comparaison avec les essais en échelon de même amplitude (fichiers `fixed2`) :

| Indicateur mesuré | Échelons | Rampes |
|---|---|---|
| Saturation | quelques % | **0 %** pour les 4 réglages, tension crête 13–17 V |
| Tangage déplacé par le lacet | 12,6–16,2° | 1,1–2,8° |
| Lacet déplacé par le tangage | 4,5–6,6° | 1,1–1,5° |
| Dépassement du lacet | 7–12 % | ≈ 1 % |
| Fit du modèle (Current) | 88,5 / 86,4 % | 92,4 / 94,5 % |
| Erreur statique de tangage | 3,1 / 6,6 / 3,2 / 3,5° | 3,0 / 6,1 / 3,2 / 3,9° (inchangée) |

Sans saturation, la boucle reste dans le domaine où le modèle linéaire est valable : c'est le « rester dans le linéaire » de l'enseignant, vérifié par la mesure. En revanche, la rampe change la façon de rejoindre la consigne, pas le point d'arrivée : **l'erreur statique ne change pas**.

### 6.2 Erreur statique et poussée en V²

Pour les 4 réglages, l'erreur statique de tangage mesurée vaut **1,9 à 2,1 fois** celle simulée. Les tensions de maintien sont aussi doublées :

| Maintien du tangage à ±20° | \|Vp\| | \|Vy\| |
|---|---|---|
| Simulation | 3,0–3,3 V | 3,7–4,0 V |
| Mesure | 5,2–6,6 V | 8,0–8,4 V |

**Explication.** La gravité (terme Ksp·θ) exige un couple permanent pour tenir le bras incliné. Un LQR sans intégrateur ne produit une tension qu'à partir d'une erreur. Si chaque volt donne moitié moins de couple que prévu, il faut deux fois plus de tension, donc deux fois plus d'erreur.

**Vérification.** En simulant avec la matrice **B × 0,5** (gains moteurs divisés par 2), on retrouve l'erreur mesurée pour les 4 réglages :

| e_ss tangage (°) | Current | Softer | More damping | Bryson |
|---|---|---|---|---|
| Mesure | 2,98 | 6,09 | 3,17 | 3,92 |
| Simulation, B nominal | 1,60 | 3,21 | 1,60 | 1,98 |
| Simulation, B × 0,5 | 3,01 | 5,84 | 2,99 | 3,71 |

Avec B × 0,5, les tensions de maintien simulées (≈ 6 / 7,3 V) correspondent aussi à la mesure, et le fit progresse sur les deux axes (Current : 94,0 / 95,7 %).

C'est cohérent avec les essais en boucle ouverte : la poussée croît environ comme **V²**, donc le couple par volt est faible à 5–9 V, alors que le B de Quanser correspond à un point de fonctionnement plus élevé. Ksp semble correct, puisque la période propre du tangage en boucle ouverte était bien prédite.

Deux détails confirment cette lecture :
- Le rapport mesuré Vy/Vp ≈ 1,25 au maintien est proche de celui du modèle (Kyp/Kyy ≈ 1,23) : la structure des couples est juste, seule leur amplitude est surestimée.
- Tenir −20° demande plus de tension que +20° (≈ 8 V contre 6,5 V, hélice à ≈ 160 contre 120 rad/s) : les hélices sont moins efficaces en marche arrière.

**Conséquence :** aucun réglage de Q et R ne supprime cette erreur. Recalculer K avec B × 0,5 et les mêmes Q, R revient exactement à 2 × K_Softer et donnerait ≈ 3,2°. Augmenter les gains (R ÷ 4 → 1,6°) fait monter K_vel jusqu'à 91, donc le bruit.

### 6.3 Bruit de dérivation

L'activité de tension mesurée est 10 à 25 fois celle simulée, et **proportionnelle au plus grand gain de vitesse** : le rapport activité / K_vel vaut 1,01, 1,06, 1,02 et 0,97 pour les 4 réglages.

Les vitesses sont obtenues en dérivant les codeurs : chaque pas codeur (0,18°) produit un pic de vitesse, multiplié par K_vel. La simulation utilise des vitesses exactes et ne voit pas ce bruit. La rampe n'y change rien. More damping est le plus bruité (70 / 90 V/s) sans bénéfice, puisqu'aucun réglage ne dépasse en tangage.

### 6.4 Frottement sec du lacet

Après le premier mouvement de lacet, l'angle dépasse jusqu'à −41,9° et **reste bloqué environ 5 s**, alors que la simulation revient à −40°. Une erreur de 2° donne 0,035 rad × 66 V/rad ≈ 2,3 V, bien en dessous du seuil de décollement de 6–7 V mesuré en boucle ouverte (zone morte). Le lacet ne se libère que lorsque le mouvement de tangage suivant lui donne une impulsion.

C'est aussi pourquoi l'erreur de lacet mesurée vaut environ 2 fois la simulation. Le modèle linéaire, qui ne contient qu'un frottement visqueux Dy, ne peut pas représenter ce phénomène.

### 6.5 Comparaison des réglages

- **Current : meilleur compromis avec les rampes.** RMSE, erreur de tangage et couplage les plus faibles, meilleur fit, bruit moyen. Avec des échelons, ses gains élevés le faisaient saturer ; sans saturation, sa raideur devient un avantage.
- **Bryson (R = 0,005) :** proche de Current en bruit, mais 1° d'erreur de tangage en plus (K₁₁ = 77 contre 98). Avec les échelons, son avantage venait surtout d'une saturation plus faible. Avec R = 0,0044, l'erreur prévue serait d'environ 3,6–3,7° : le classement ne changerait pas.
- **More damping :** la meilleure erreur de lacet (0,82°), mais le plus bruité. Son amortissement supplémentaire n'est pas utile en tangage.
- **Softer :** le moins bruité, mais une erreur de 6,1° : rejeté.

### 6.6 Limites du protocole

- **Un seul essai par réglage :** la dispersion d'un essai à l'autre est inconnue, donc des écarts inférieurs à environ 0,5° ne sont pas significatifs.
- **Le dernier mouvement de lacet se termine à 29 s :** il ne reste qu'1 s, et son « erreur statique » (3–8°) est en réalité du retard. La médiane sur les 3 mouvements la masque de justesse.
- **Couplage fragile.** θ←ψ repose sur un seul mouvement (12,5 s), coupé à 15 s alors que la rampe de lacet dure jusqu'à 16,5 s. ψ←θ à 5 s est gonflé par le lacet qui se libère de son blocage. Les écarts de 1–2° entre réglages ne sont donc pas fiables.
- **t_r et t_s mesurent surtout la rampe :** ils ne servent qu'à vérifier que l'angle la suit sans retard notable.

## 7. Prochaines étapes

1. **Corriger B dans la simulation** avec un facteur séparé, `Bscale = 0.5; B = Bscale*B;`, valable vers 5–9 V. Le fichier Quanser reste inchangé, et les deux modèles peuvent être comparés.

2. **Gain de pré-contrôle** (piste 4 de l'enseignant). On ajoute à la commande la tension qui maintient l'équilibre en x = [r; 0; 0] :

   u = K (x_ref − x) + N_u · r, avec N_u = −B₃:₄⁻¹ A₃:₄ [I; 0]

   Ce qui donne :
   - N_u = [10,19 0 ; 12,50 0] V/rad avec le modèle nominal ;
   - N_u = [20,4 0 ; 25,0 0] V/rad avec B × 0,5.

   La deuxième colonne est nulle parce que le lacet n'a pas de ressort : tenir un angle de lacet ne coûte aucune tension. Forme générale équivalente : N = [C (BK − A)⁻¹ B]⁻¹ = K(:,1:2) + N_u.

   En simulation, sur le « procédé réel » (B × 0,5) avec le K de Current :

   | Pré-contrôle | e_ss tangage | Dépassement |
   |---|---|---|
   | Aucun | 3,01° | 0 % |
   | N_u nominal | 1,54° | 0,5 % |
   | N_u avec B × 0,5 | **0,06°** | 4,5 % |
   | N_u surestimé (B × 0,4) | −0,68° (dépasse) | 6,4 % |

   Le pré-contrôle ne vaut que ce que vaut le modèle. Il n'ajoute ni état, ni bruit, ni emballement d'intégrateur, mais il ne rejette pas les perturbations, et la poussée en V² ainsi que l'asymétrie ± laisseront une erreur résiduelle d'environ 0,5–1°.

   **Essai proposé :** Current + N_u (B × 0,5) sur l'AERO. Il suffit d'un bloc Gain (`Matrix(K*u)`, entrée = consigne en rad après le Rate Limiter) et d'un bloc Sum avant la saturation ±20 V.

3. **LQI avec anti-windup, combiné au pré-contrôle.** Le pré-contrôle fournit l'essentiel de la tension de maintien, et l'intégrateur corrige le reste, ce qui permet un intégrateur plus lent et moins sujet à l'emballement. À surveiller : en lacet, l'intégrateur peut provoquer un **stick-slip** (blocage, décollement, dépassement, nouveau blocage) à cause du frottement sec.

4. **Réduire le bruit de dérivation :** filtre passe-bas sur θ̇ et ψ̇, ou observateur (Kalman / LQG). Il faut d'abord vérifier le bloc qui calcule les vitesses dans Simulink, puis l'ajouter à la simulation.

5. **Améliorer le protocole :**
   - un essai où seul le tangage bouge et un essai où seul le lacet bouge, avec au moins 3–4 s de repos après chaque rampe, pour mesurer proprement le couplage ;
   - ou un essai de 40 s avec des mouvements décalés ;
   - répéter au moins un réglage pour connaître la dispersion.

6. **Optionnel :** affiner le modèle gris (B dépendant du point de fonctionnement, frottement sec du lacet), puis comparer avec un PID découplé (piste 3).

## 8. Conclusion

La consigne en rampe (Rate Limiter à 20 °/s, consigne bornée et démarrant à 0) a supprimé la saturation, divisé le couplage par 3 à 15 et le dépassement du lacet par environ 10, et amélioré la prédiction du modèle (fit 92–95 %).

L'erreur statique de tangage (≈ 3° pour Current) reste. Elle s'explique entièrement par l'absence d'action intégrale et par des moteurs qui fournissent environ moitié moins de couple par volt que le modèle à faible tension (poussée en V²).

Le lacet est en plus limité par le frottement sec, et la commande par le bruit de dérivation. Avec les rampes, **Current** est le meilleur compromis. La suite logique est : correction de B, pré-contrôle, puis LQI.

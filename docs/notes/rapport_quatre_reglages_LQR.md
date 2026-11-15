# Essais des quatre réglages LQR sur le Quanser AERO

Mini-projet Quanser AERO, Polytech Nancy, 5 octobre 2026

## Essais

Quatre réglages, conçus et simulés avec `tune_lqr_AERO.m`, ont été testés sur la maquette avec les mêmes consignes : tangage ±20° toutes les 5 s, lacet ±40° toutes les 12,5 s, sur 30 s.

| Fichier | Réglage | $Q$ | $R$ |
|---|---|---|---|
| `aero_lqr_rsp_fixed2_sys1` | Actuel | diag(200, 75, 0, 0) | 0,005 · I |
| `aero_lqr_rsp_fixed2_sys2` | Plus doux | diag(200, 75, 0, 0) | 0,02 · I |
| `aero_lqr_rsp_fixed2_sys3` | Plus amorti | diag(200, 75, 20, 10) | 0,005 · I |
| `aero_lqr_rsp_fixed2_sys4` | Bryson (5°, 10°, 60°/s, 15 V) | diag(131 ; 33 ; 0,91 ; 0,91) | 0,0044 · I |

## Résultats mesurés

| Réglage | $t_r$ tangage | Erreur tangage | Dépassement lacet | Tangage déplacé par le lacet | Lacet déplacé par le tangage | Activité des tensions |
|---|---|---|---|---|---|---|
| Actuel | 0,69 s | 3,1° | 9,2 % | 16,2° | 6,6° | 63 V/s |
| Plus doux | 0,90 s | 6,6° | 12,2 % | 13,2° | 4,5° | 43 V/s |
| Plus amorti | 0,99 s | 3,2° | 6,8 % | 13,6° | 5,0° | 83 V/s |
| Bryson | 0,75 s | 3,5° | 10,8 % | 12,6° | 6,2° | 53 V/s |

Les valeurs par échelon sont des médianes. L'activité des tensions est $\sum |\Delta V| / T$, moyennée sur les deux moteurs.

![Comparaison mesure et simulation](four_designs_metrics.png)

## Constats

1. **Erreur en tangage deux fois plus grande qu'en simulation, pour les quatre réglages.** Le rapport mesure/simulation vaut entre 1,9 et 2,1. Aux faibles tensions, les moteurs réels donnent environ la moitié du couple prévu par le modèle (poussée en $V^2$).
2. **Lacet trop peu amorti.** Le dépassement mesuré est de 7 à 12 %, contre 1 à 2 % en simulation. L'amortissement du lacet dans le modèle est trop fort, et le frottement sec arrête le lacet avant la consigne.
3. **Couplages sous-estimés par le modèle.** Le tangage est déplacé 1,5 à 1,8 fois plus que prévu, et le lacet 4 à 5 fois plus.
4. **Bruit proportionnel au gain en vitesse.** L'activité des tensions suit l'ordre des gains en vitesse : Plus doux (30 V·s/rad), Bryson (44), Actuel (46), Plus amorti (69). En simulation, elle reste entre 13 et 16 V/s, car les vitesses n'y sont pas dérivées des codeurs.
5. **Modèle plus fidèle avec des échelons plus petits.** Pour le réglage actuel, le fit passe de 83 à 88,5 % en tangage, car les moteurs saturent moins.

## Choix

- **Plus doux** est écarté (6,6° d'erreur).
- **Plus amorti** donne le plus faible dépassement en lacet, mais c'est le plus lent et le plus bruité.
- **Actuel** est le plus rapide, mais c'est celui qui couple le plus.
- **Bryson** est le meilleur compromis : le couplage le plus faible, le deuxième plus rapide, un bruit modéré.

Chaque réglage n'a été essayé qu'une fois. Un second essai d'un même réglage donnerait la dispersion d'un essai à l'autre.

## Suite

1. Recaler le lacet du modèle ($D_y$, puis $K_{yy}$ et $K_{yp}$).
2. Ajouter une action intégrale (LQI) avec anti-emballement, à partir du réglage de Bryson, pour supprimer l'erreur en tangage et le blocage du lacet.

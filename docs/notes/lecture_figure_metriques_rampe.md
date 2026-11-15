# Lecture de la figure « Ramp tests: metrics »

**Lecture :** 9 sous-figures, une par critère. Pour chaque réglage (Current, Softer, More damping, Bryson), la barre de gauche correspond au **tangage** et celle de droite au **lacet**, mesurés sur l'AERO. Les **croix noires** donnent la simulation (modèle Quanser). **Plus bas = meilleur** partout. Les valeurs sont des médianes sur les mouvements de consigne.

1. **Tracking RMSE (°)** : écart quadratique moyen entre consigne et angle sur tout l'essai. Résume la qualité de suivi globale, transitoires compris. Meilleur : Current (3,9 / 5,3°).
2. **Overshoot (%)** : dépassement maximal au-delà de la cible, en % de l'amplitude du mouvement. Mesure l'amortissement. Nul en tangage et ≈ 1 % en lacet pour les 4 réglages, grâce à la rampe.
3. **Steady-state error (°)** : écart entre consigne et angle moyen sur la dernière seconde de chaque palier. C'est la précision finale. ≈ 2× la simulation en tangage, à cause du couple réel par volt plus faible (poussée en V²) ; frottement sec en lacet. Meilleur : Current en tangage (2,98°).
4. **RMS voltage (V)** : tension efficace de chaque moteur, c'est-à-dire l'effort de commande. ≈ 2× la simulation, pour la même raison que le critère 3.
5. **Time saturated (%)** : part du temps passée à ±20 V. Au-delà de quelques %, la boucle n'est plus linéaire. 0 % pour les 4 réglages, grâce à la rampe.
6. **Voltage activity Σ|dV|/T (V/s)** : variation totale de la tension par seconde, qui mesure le bruit de commande. Proportionnelle au plus grand gain de vitesse (bruit de dérivation des codeurs), 10 à 25× la simulation. Meilleur : Softer ; pire : More damping.
7. **Moved by the other axis (°)** : écart maximal d'un axe pendant un mouvement de l'autre seul. C'est le couplage. Peu fiable ici : 1 à 3 mouvements seulement par mesure.
8. **Rise time 10–90 % (s)** : temps de montée. Avec une rampe, il mesure surtout **la durée de la rampe** (≈ 1,6 s pour 40°), pas la rapidité du correcteur. Sert seulement à vérifier que l'angle suit la rampe sans retard.
9. **Settling time (s)** : temps après lequel l'angle reste dans ±5 % de sa valeur finale. Comme le critère 8, il **inclut la rampe**. Utile seulement pour repérer une oscillation résiduelle.

**Critères décisifs avec une rampe :** erreur statique (3), saturation (5), activité de tension (6) et dépassement (2). Les critères 8 et 9 ne servent pas à départager les réglages.

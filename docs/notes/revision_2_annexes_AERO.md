# Révision 2 — consignes validées (version finale)

*À donner à Claude dans PowerPoint avec `Presentation_Rayan_Pierre_2.pptx`.*

Ne pas modifier :

- la diapo 8 ;
- la diapo 18 ;
- l'annexe A0 ;
- l'infobulle de la diapo 16 (elle reste).

L'ancienne annexe A1 reste supprimée.

---

## 1. Diapo 16 — PD découplé : résultats

1. Dans la sous-puce sous « Précision : le PD gagne », **retirer le renvoi à l'annexe A3** (supprimée). Nouveau texte :
   > L'essai LQR n'avait pas de pré-commande $N_u$ : c'est la principale raison de l'écart.
2. Remplacer la puce « Rapidité : le LQR est un peu plus rapide (0,38 s contre 0,44 s) » par :
   > **Le modèle identifié prédit aussi le PD** : fit 90 / 96 % et erreur restante prédite au centième de degré près (annexes A7–A8).

---

## 2. Annexes

### Supprimer

- **Annexe A3** (« gain B × 0,5 et pré-commande »).
- **Annexe A14 (suite)** (« résultats de l'identification »), après en avoir récupéré le diagramme et les puces (voir A5 et A14 ci-dessous).

### Modifier

**A2 — LQR, réglages et matrice K**

- Supprimer ce qui répète la diapo 7 :
  - l'encadré « Modèle de Quanser » ;
  - le schéma bloc ;
  - les lignes J et u = K(x_ref − x).
- Garder :
  - l'équation de Riccati et K = R⁻¹BᵀP ;
  - les 4 réglages (Q, R) ;
  - la matrice K et la phrase sur les termes hors diagonale.
- Nouveau sous-titre :
  > « Ce que la diapo 7 ne détaille pas : réglages, Riccati, gain K »

**A4 — vitesses et bruit**

Supprimer la ligne « Source : … ».

**A5 — identification, signaux et paramètres**

- Supprimer l'encadré des 4 fonctions de transfert (déjà sur la diapo 11).
- Dans l'espace vide à droite, coller depuis l'ancienne A14 (suite) :
  - le diagramme « Fit des fonctions de transfert identifiées » ;
  - ses 2 puces « G12 le plus faible » et « Même corps, pôles différents ».

**A14 — paramètres complets**

Ajouter dans la colonne de droite les 2 puces de l'ancienne A14 (suite) :

- « Pourquoi l'écart » ;
- « Quanser : K_py ≈ 1,9 K_pp ; identifié 0,8 à 0,9 K_pp ».

**A7 (suite) — pourquoi le lacet reste imparfait**

Dans le tableau « Vérification », supprimer la ligne « Fit vitesse du lacet | 87 % | 74 % (linéaire) ». Elle place un fit de modèle dans la colonne « Mesure », et ce résultat est déjà sur la diapo 14.

**A8 — PD, loi de commande et gains**

- Supprimer :
  - la matrice numérique « = [540 −933 ; 982 1123] V/(N·m) » (garder W = B_t⁻¹) ;
  - le tableau des gains (k_p, k_d) ;
  - la note interne « Ne jamais relancer le script Quanser après le setup (gains en V) ».
- Garder tout le reste.

**A10 — le couple manquant**

Garder le tableau, la formule et le diagramme. Remplacer les 4 puces par 2 :

- « η plus faible à basse tension (63 % vers 5 V, 78 % vers 10 V) : poussée en V² et gains croisés du modèle optimistes. »
- « L'anticipation fournit 82 à 88 % du couple de maintien et divise l'erreur par ≈ 4,5 ; le reste est le couple que les moteurs ne donnent pas : seule une action intégrale l'annule. »

**A11 — le réglage cKyp**

- **Garder la formule de W.**
- Supprimer le tableau « cKyp | Dosage Vy/Vp | Tenir 20° ».
- Ajouter sous la formule :
  > « cKyp multiplie le gain croisé K_yp utilisé par le découpleur : il change la proportion entre les deux moteurs. »
- Garder les 3 puces et l'encadré Répétabilité.
- Ne garder que le 3ᵉ diagramme (**Couplage**) : les deux autres répètent le tableau de la diapo 16.

---

## 3. Ordre et nouvelle numérotation des annexes

Après les suppressions, déplacer et renommer les annexes pour qu'elles suivent l'ordre de l'exposé, sans trou dans la numérotation :

| Nouveau nom | Ancien nom | Contenu |
|---|---|---|
| A0 | A0 | Critères d'évaluation (inchangée) |
| A1 | A2 | LQR : réglages et matrice K |
| A2 | A4 | Mesure des vitesses et bruit |
| A3 | A5 | Identification : signaux et fits |
| A3 (suite) | A14 | Paramètres complets |
| A4 | A6 | Test de linéarité : déroulement de l'essai |
| A4 (suite) | A13 | Linéaire ou non linéaire ? |
| A5 | A7 | Frottement, validation à 10 V |
| A5 (suite) | A7 (suite) | Pourquoi le lacet reste imparfait |
| A6 | A8 | PD : loi de commande |
| A7 | A9 | PD : simulation et mesure |
| A8 | A10 | Le couple manquant |
| A9 | A11 | Le réglage cKyp |
| A10 | A12 | Limites physiques du PD découplé |

**Renvois à mettre à jour :**

- **Diapo 16** : « annexes A7–A8 » (déjà écrit ainsi au § 1).
- **Nouvelle A7** (ancienne A9), dernière puce : « annexe A10 » → « **annexe A8** ».

---

## 4. Carte « question → annexe » (à imprimer)

En mode diaporama, **taper le numéro de la diapo puis Entrée** pour y aller directement. Pour revenir, taper le numéro de la diapo « Merci » puis Entrée. Compléter la colonne « N° » une fois l'ordre final fixé.

| Question probable | Où répondre | N° |
|---|---|---|
| Comment avez-vous choisi vos critères ? | A0 | |
| Quelles valeurs de Q, R ? Comment K est-il calculé ? | A1 | |
| Comment mesurez-vous les vitesses ? D'où vient le bruit ? | A2 | |
| Quels essais pour identifier ? Pourquoi 8 V ? Pourquoi procsrivc ? | A3 | |
| Valeurs des paramètres ? Pourquoi G12 à 60 % ? | A3 (suite) | |
| Comment s'est déroulé le test de linéarité ? | A4 | |
| Finalement, le système est-il linéaire ? | A4 (suite) | |
| Le modèle avec frottement marche-t-il à 10 V ? Comment estimer F_c ? | A5, A5 (suite) | |
| Comment avez-vous réglé le PD ? Pourquoi inverser B_t ? | A6 | |
| Le modèle prédit-il aussi le PD ? | A7 | |
| D'où vient l'erreur restante du PD ? | A8 | |
| Qu'est-ce que cKyp ? Vos essais sont-ils répétables ? | A9 | |
| Pourquoi le découplage n'est-il pas meilleur ? | A10 | |
| Le PD a une anticipation, pas le LQR : est-ce juste ? | **à l'oral** (voir ci-dessous) | — |
| Quelles difficultés avez-vous rencontrées ? | **à l'oral** (voir ci-dessous) | — |

### Réponses orales (questions sans annexe)

**Anticipation PD contre LQR.**

> « Non, ce n'est pas tout à fait juste, et c'est la principale raison de l'écart. Le LQR peut recevoir la même anticipation, une pré-commande $N_u$ qui fournit la tension de maintien du tangage. En simulation, LQR + $N_u$ ramène l'erreur de 3° à 0,06°. Nous ne l'avons pas testée sur la maquette faute de temps. »

**Difficultés.**

> « Les lignes du fichier n'étaient pas documentées. On enregistrait les courants au lieu des tensions. Il y avait une erreur degrés/radians dans le bloc State X : les gains étaient multipliés par 57, et la commande passait en tout-ou-rien. Le gain du codeur de lacet était faux (512 au lieu de 1024). Des timeouts HIL ont été résolus par la saturation à ±20 V. Enfin, Excel tronquait les données à 16 384 colonnes. Nous avons appris à vérifier chaque signal par la résolution des codeurs avant de l'exploiter. »

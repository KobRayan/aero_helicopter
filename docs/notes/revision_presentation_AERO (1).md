# Révision de la présentation AERO — consignes diapo par diapo

*À donner à Claude dans PowerPoint avec la présentation `Presentation_Rayan_Pierre.pptx`.*
*Soutenance : 16/10/2026, 15 min d'exposé + 5 min de questions, 2 orateurs (Pierre Baudin, Rayan Kobrossly).*

---

## 0. Contexte pour Claude dans PowerPoint

Mini-projet de commande d'un hélicoptère à 2 degrés de liberté (Quanser AERO) : 2 tensions d'entrée ($V_p$, $V_y$), 2 angles de sortie (tangage $\theta$, lacet $\psi$).

Le fil rouge de l'exposé est le suivant :

1. Un premier modèle (Quanser) et une première commande (LQR).
2. Les écarts constatés sur la maquette.
3. Un modèle ré-identifié (CONTSID), avec ses non-linéarités.
4. Une seconde commande (PD découplé).

**Problème actuel :** 22 diapos de contenu pour 15 minutes, beaucoup de nombres bruts et de tableaux, et 4 diapos ont encore des cadres « Figure à insérer ».

**Objectif :** environ 14 diapos de contenu (≈ 1 min chacune), **une idée par diapo**, une figure par diapo, et **au plus 3 nombres** dans le corps d'une diapo. Tout le détail chiffré part en annexe, où il sert pour les questions.

### Règles à appliquer partout

- Le titre-message (sous-titre bleu) reste : c'est la phrase que le jury doit retenir.
- **3 puces maximum** par diapo, 1 ligne chacune si possible.
- **Arrondir** les nombres : 2,98° → 3° ; 12,6–16,2° → « 13 à 16° » ; 92,4 / 94,5 % → « > 90 % ».
- **Pas de tableau de plus de 3 lignes** dans le corps de l'exposé. Les tableaux complets vont en annexe.
- Ne pas afficher de matrices de gains ni de noms de fichiers ou de scripts (`fixed3_sys1`, `tune_lqr_AERO.m`…). Les noms de scripts peuvent aller en note de bas de page en petit, ou être supprimés.
- Les figures MATLAB avec des titres en anglais : remplacer le titre de la figure par un titre français dans la diapo (*measured* = mesure, *yaw rate* = vitesse de lacet, *pitch* = tangage, *steady-state error* = erreur statique).

---

## 1. Nouvelle structure (≈ 14 diapos de contenu)

| Nouv. | Ancienne(s) | Titre | Orateur | Action |
|---|---|---|---|---|
| 1 | 1 | Titre | — | inchangé |
| 2 | 2 | Sommaire | — | inchangé |
| 3 | 3 | Intercalaire « Le système et la première commande » | Pierre | inchangé |
| 4 | 4 (+5) | Objectifs et démarche | Pierre | ajouter 1 ligne de critères ; diapo 5 → annexe |
| — | 6 | La maquette et la chaîne d'acquisition | — | **→ annexe** (fusionner avec A1) |
| 5 | 7 | Essais en boucle ouverte | Pierre | alléger le texte, insérer les 2 figures |
| 6 | 8 | Les non-linéarités | Pierre | **1 seule figure** au lieu de 3 cadres vides |
| 7 | 9 | Commande LQR : principe | Pierre | supprimer la matrice K et les détails des réglages |
| 8 | 10 | LQR : simulation et réalité | Pierre | 2 chiffres au lieu de 4, insérer la figure |
| 9 | 11 | Consigne en rampe | Pierre | tableau réduit à 3 lignes |
| 10 | 12 | Intercalaire « Un modèle plus fidèle, une seconde commande » | Rayan | inchangé |
| 11 | 13 + 14 | Identification du modèle linéaire | Rayan | **fusion** : formules + figure G11/G22 |
| 12 | 15 + 16 | Résultats et validation du modèle | Rayan | **fusion** : garder le diagramme en barres |
| 13 | 19 | Test de linéarité (homogénéité) | Rayan | **nouvelle figure 4 panneaux** |
| 14 | 17 + 18 | Frottement sec | Rayan | **fusion**, 1 équation + 2 vérifications |
| 15 | 20 (+21) | PD découplé : principe | Rayan | garder le schéma ; diapo 21 → annexe |
| 16 | 22 | PD découplé : résultats | Rayan | tableau réduit à 3 lignes |
| 17 | 23 | Conclusion | Rayan | légère réduction |
| 18 | 24 | Perspectives et retour d'expérience | Rayan | 4 pistes + 1 encadré « avec le recul » |
| 19 | 25 | Merci | — | inchangé |
| A… | 5, 6, 21, 26–34 | Annexes | — | conserver |
| (masquée) | **35** | Liens des conversations Claude | — | **Masquer la diapositive** |

Temps cible : Pierre ≈ 7 min (diapos 3–9), Rayan ≈ 7 min 30 (diapos 10–18).

### Pour que l'exposé ne traîne pas

- **1 minute par diapo de contenu.** Si une diapo demande plus de 3 phrases à l'oral, elle est trop chargée.
- **Ne pas lire la diapo.** Dire le titre-message, montrer *une* chose sur la figure (« regardez ici… »), puis passer à la suite.
- **Les non-linéarités tiennent en 2 diapos (6 et 13), pas 4.** La figure 1 (déroulement de l'essai) va en annexe.
- **Si l'on dépasse en répétition, couper dans cet ordre :**
  1. diapo 14 (frottement) → à résumer en une phrase sur la diapo 13 : « Un terme de frottement sec reproduit la zone morte : décollage prédit à 5,8 V, mesuré entre 6 et 7 V » ;
  2. diapo 8 (LQR simulation et réalité) → fusionner avec la diapo 9 ;
  3. diapo 5 (boucle ouverte) → ne garder que la figure de superposition.
- **Répéter une fois à deux avec un chronomètre**, cible 14 min, pour garder une marge.

---

## 2. Consignes diapo par diapo

### Diapo 4 — Objectifs et démarche (ancienne 4, + 5)

- Garder le cycle Modèle → Commande → Essai → Analyse des écarts.
- Remplacer les trois sous-puces « Objectifs donnés » par une seule ligne : *« Valider le modèle Quanser, identifier un modèle (CONTSID), synthétiser un LQR, puis aller plus loin. »*
- Ajouter une puce : **« Pas de cahier des charges imposé : nous avons comparé les correcteurs sur 4 critères mesurés — erreur statique, couplage entre axes, saturation, bruit de commande. »**
- Ancienne diapo 5 (tableau des 7 indicateurs) → **annexe A0**.

### Diapo 5 — Essais en boucle ouverte (ancienne 7)

- Garder 3 puces :
  - *« 7 échelons : 5, 10, 15 V sur chaque moteur, puis les deux à 10 V. »*
  - *« Le moteur de lacet fait basculer le tangage autant que le moteur de tangage lui-même (couple de réaction des hélices). »*
  - *« La superposition n'est vraie que pendant 2 s : le système n'est linéaire que localement. »*
- Supprimer « pic de 2–3 °/V » et « 31° contre 17° » du texte : la figure de superposition le montre.
- Retirer l'encadré $G(s)$ (il revient en diapo 11).
- Figures : `TOR_cross-coupling_per_volt.png` et `TOR_superposition.png`. Elles viennent du rapport de boucle ouverte, et se régénèrent avec `plot_all_steps_AERO.m` (figures 3 et 4).

### Diapo 6 — Les non-linéarités (ancienne 8)

- **Remplacer les 3 cadres vides par une seule figure : `2_caracteristique_statique.png`.** Elle montre à elle seule la zone morte, la croissance en V², la dissymétrie et l'écart au modèle linéaire (≈ 5× à 16 V).
- À gauche, 3 lignes courtes (sans les nombres 0,26 / 0,255 / 0,22) :
  1. **Poussée ∝ V²** : $F = k_f\,\omega^2$ avec $\omega \propto V$, donc le gain dépend de la tension.
  2. **Zone morte** (frottement sec) : le lacet ne bouge pas sous 6 V.
  3. **Effet centrifuge** : le tangage retombe quand le lacet tourne vite (négligeable avec nos consignes lentes).
- Phrase à dire : *« Linéariser, c'est prendre la tangente de cette courbe en un point ; un modèle linéaire n'est juste qu'autour de ce point. »*

### Diapo 7 — Commande LQR : principe (ancienne 9)

- Garder : le modèle de Quanser (2 équations), $J = \int (x^TQx + u^TRu)\,dt$, $u = K(x_{ref}-x)$, et le schéma bloc.
- **Supprimer** la matrice $K$ chiffrée, l'équation de Riccati et la liste des 4 réglages.
- Remplacer par une ligne : *« 4 réglages de Q et R comparés ; le réglage retenu donne le meilleur compromis précision / bruit. »*
- Garder l'idée : *« $K$ a des termes croisés : le LQR utilise les deux moteurs pour chaque axe. »*

### Diapo 8 — LQR : simulation et réalité (ancienne 10)

- Garder **2 chiffres seulement** :
  - **« ×2 »** : erreur statique de tangage deux fois plus grande que prévue (≈ 3°) ;
  - **« 13 à 16° »** : tangage déplacé par un échelon de lacet.
- Supprimer les blocs « 7–12 % » et « 43–83 V/s » (ils vont en annexe A4).
- Figure : un essai LQR en échelons, mesure + simulation superposées (`plot_lqr_AERO.m` ou `tune_lqr_AERO.m` sur l'essai en échelons ±20° / ±40°, réglage retenu). Si la figure n'est pas prête, agrandir les 2 blocs et retirer le cadre vide.

### Diapo 9 — Consigne en rampe (ancienne 11)

- Tableau réduit à 3 lignes, nombres arrondis :

| Mesuré | Échelons | Rampes 20 °/s |
|---|---|---|
| Saturation | quelques % | **0 %** |
| Tangage déplacé par le lacet | 13 à 16° | **1 à 3°** |
| Erreur statique de tangage | 3° | 3° (inchangée) |

- À droite, 2 puces seulement :
  - *« L'erreur reste : sans intégrateur, il faut une erreur pour produire la tension qui tient le tangage contre la gravité. »*
  - *« Les moteurs donnent ≈ 2× moins de couple par volt que le modèle (poussée en V²) : la simulation avec B ÷ 2 retrouve l'erreur mesurée. »*
- Supprimer : la ligne « Fit », la ligne « Réalisation : Rate Limiter… » et la piste pré-commande (déjà en annexe A3).

### Diapo 11 — Identification du modèle linéaire (anciennes 13 + 14)

- En haut, garder les 4 fonctions de transfert, en LaTeX :

```
G_{11}=\frac{K_{pp}}{J_p s^2+D_p s+K_{sp}},\quad G_{12}=\frac{K_{py}}{J_p s^2+D_p s+K_{sp}},\quad G_{21}=\frac{K_{yp}}{s(J_y s+D_y)},\quad G_{22}=\frac{K_{yy}}{s(J_y s+D_y)}
```

- 2 puces :
  - *« Structure physique de Quanser conservée ; seuls ses paramètres sont ré-identifiés avec CONTSID (SRIVC, temps continu). »*
  - *« Essais en créneaux ±8 V, au-dessus de la zone morte et dans le domaine utilisé par nos correcteurs. »*
- Figure : celle de l'ancienne diapo 14, **en gardant seulement G11 et G22** (recadrer), avec le titre « Mesure (noir) et modèle identifié (bleu) ».
- Supprimer : le tableau des signaux et l'encadré de correspondance des paramètres (→ annexe).

### Diapo 12 — Résultats et validation du modèle (anciennes 15 + 16)

- Titre-message : *« Le modèle identifié prédit l'erreur réelle du LQR ; le modèle Quanser la sous-estime de 40 %. »*
- **Figure principale : le diagramme en barres de l'ancienne diapo 16** (mesuré / simulé Quanser / simulé identifié). Traduire le titre et la légende.
- 3 puces :
  - *« Les moteurs réels donnent 1,5 à 6 fois moins de couple par volt que le modèle Quanser (identifié à 20 V, la poussée en V² explique l'écart). »*
  - *« Fits : 82 à 96 %, sauf le couplage G12 (60 %) : le couple de réaction de l'hélice n'est pas dans la structure. »*
  - *« La boucle fermée cache les erreurs de modèle : c'est l'erreur statique qui départage les modèles. »*
- Le tableau des paramètres et le diagramme des fits → **annexe A10** (qui existe déjà).

### Diapo 13 — Test de linéarité : homogénéité (ancienne 19)

- **Mise en page : 2 figures côte à côte**, `3_test_homogeneite.png` à gauche et `4_origine_non_linearite.png` à droite.
- Titre-message : *« Le gain du lacet n'est pas constant ; la non-linéarité vient de l'hélice et du frottement, pas du moteur. »*
- 1 ligne au-dessus des figures : *« Chaque réponse est divisée par sa tension : si le système était linéaire, toutes les courbes seraient identiques. »*
- 2 puces maximum sous les figures :
  - *« K passe de 0 (5 V) à 5,7 °/s/V (13 V) ; le modèle linéaire donne 1,6. »*
  - *« Le moteur est linéaire (vitesse d'hélice ≈ 16 rad/s par volt) ; le lacet suit ω² (poussée) moins un seuil de frottement sec. »*
- Retirer l'équation $0{,}68\,(V^2-50)$.
- `1_essai_impulsions.png` (déroulement de l'essai) → **annexe**. On peut la montrer si le jury demande comment l'essai a été fait.

### Diapo 14 — Modèle non linéaire : frottement sec (anciennes 17 + 18)

- Titre-message : *« Ajouter un frottement sec explique la zone morte et les arrêts du lacet. »*
- 1 équation seulement :

```
J_y\ddot\psi + D_y\dot\psi + F_{c,y}\,\mathrm{sign}(\dot\psi) = K_{yy}V_y
```

- 2 vérifications (petit tableau à 2 lignes) :

| | Modèle | Mesure |
|---|---|---|
| Tension de décollage du lacet | 5,8 V | 6–7 V |
| Bande de blocage du tangage | ±1° | +1,4 / −1,8° |

- 1 puce : *« Estimé par moindres carrés non linéaires sur la simulation complète ; fit de la vitesse de lacet de 74 à 87 %. »*
- Figure : **les 2 panneaux du haut** de l'ancienne diapo 18 (estimation : angle et vitesse), titre « Mesure, modèle linéaire, modèle avec frottement ».
- Supprimer : $D_{eq}$, la liste « pourquoi le lacet reste imparfait », la validation à 10 V (→ annexe A9).

### Diapo 15 — PD découplé : principe (ancienne 20, + 21)

- Garder le schéma tel quel : il est très clair.
- Sous le schéma, une seule ligne de formules :

```
V = B_t^{-1}\,\tau,\qquad \tau_p = k_{p,p}(r_p-\theta) - k_{d,p}\dot\theta + K_{sp}r_p,\qquad \tau_y = k_{p,y}(r_y-\psi) - k_{d,y}\dot\psi
```

- 1 phrase : *« Chaque PD est réglé par placement de pôles (dépassement 7,5 %, pic à 1,75 s) ; l'anticipation $K_{sp}r_p$ fournit le couple contre la gravité. »*
- Ancienne diapo 21 → **annexe** (matrice W, RGA, tableau des gains).

### Diapo 16 — PD découplé : résultats (ancienne 22)

- Garder les 2 figures (elles sont très bonnes).
- Tableau réduit à 3 lignes, avec les deux colonnes LQR et PD (cK = 0,75) seulement :

| Mesuré | LQR | PD découplé |
|---|---|---|
| Erreur statique de tangage | 2,7° | **1,5°** |
| Lacet déplacé par le tangage | 1,3° | 1,7° |
| Saturation | 0 % | 0 % |

- Garder les 3 puces « Précision / Rapidité / Couplage » en raccourcissant la 3ᵉ : *« Couplage : pas de gain réel — le découpleur dépend des gains croisés, les moins bien identifiés. »*

### Diapo 17 — Conclusion (ancienne 23)

Garder le cycle « 2 tours ». Réduire à 4 lignes courtes :

- **Système** : MIMO, couplé, non linéaire, donc un modèle linéaire n'est valable que localement.
- **LQR** : fonctionne ; la rampe supprime la saturation.
- **Modèle identifié** : prédit l'erreur réelle ; le frottement explique les blocages.
- **PD découplé** : erreur statique ÷ 2 ; découplage limité par le modèle.

### Diapo 18 — Perspectives et retour d'expérience (ancienne 24)

Garder 4 pistes, sans formules :

1. **Action intégrale** (LQI / PID avec anti-windup) contre l'erreur statique restante.
2. **Compenser les non-linéarités** : linéariser la poussée, compenser le frottement sec.
3. **Estimer les vitesses** : observateur (Luenberger ou Kalman) pour réduire le bruit de commande.
4. **Affiner le découpleur** à partir des essais.

Ajouter un petit encadré **« Avec le recul »** (2–3 lignes) :

- *Fixer un cahier des charges dès le départ.*
- *Répéter chaque essai et faire bouger un seul axe à la fois.*
- *Mieux répartir le travail à 4 sur une seule maquette.*

### Diapo 35 — liens des conversations Claude

Garder la diapo dans le fichier, mais la **masquer** : clic droit sur la miniature → « Masquer la diapositive ». Elle reste dans le .pptx et ne s'affiche pas en mode diaporama.

---

## 3. Annexes (après « Merci »)

Ordre conseillé :

- **A0** : critères d'évaluation (ancienne diapo 5).
- **A1** : acquisition + difficultés (anciennes 6 + 26).
- **A2** : LQR, détail des 4 réglages et de la matrice K (contenu retiré de la diapo 7).
- **A3** : B × 0,5 et pré-commande.
- **A4** : vitesses et bruit (ajouter « dépassement du lacet 7–12 %, activité 43–83 V/s »).
- **A5** : identification, signaux et correspondance des paramètres (retirés de la diapo 11).
- **A6** : test de linéarité, déroulement de l'essai (`1_essai_impulsions.png`).
- **A7** : frottement, validation à 10 V (ancienne figure 18 complète) + $D_{eq} = 4F_c/(\pi\omega A)$.
- **A8** : PD, loi de commande et gains (ancienne 21).
- **A9 à A14** : les annexes PD et modèle déjà présentes (simulation vs mesure, couple manquant, cKyp, limites, linéaire ou non, paramètres complets).

---

## 4. Figures du test de linéarité du lacet (impulsions 0 → X V)

| Fichier | Diapo | Contenu |
|---|---|---|
| `2_caracteristique_statique.png` | 6 | Vitesse de lacet atteinte en fonction de $V_y$ : zone morte, croissance en V², dissymétrie, modèle linéaire |
| `3_test_homogeneite.png` | 13 (gauche) | Réponses divisées par la tension : une seule courbe si linéaire |
| `4_origine_non_linearite.png` | 13 (droite) | Moteur linéaire (vitesse d'hélice ∝ V), lacet ∝ ω² moins un seuil |
| `1_essai_impulsions.png` | annexe A6 | Déroulement de l'essai : $V_y$, lacet, tangage |

Précisions, si une question vient :

- **Zone morte :** de −6 à +6 V, le lacet ne *tourne* pas. Il bouge au plus de 2 à 3° à ±5–6 V (secousse due au couplage avec le tangage), puis s'arrête.
- **Couplage vers le tangage :** visible dès **2 V** (≈ 2,5° crête à crête). À 1 V, le tangage ne bouge que d'un pas codeur (0,18°), donc ce n'est pas significatif.
- **Moteur :** la vitesse d'hélice suit une droite de pente ≈ 16 rad/s par volt. Le rapport ω/V n'est pas exactement constant (21 → 17 rad/s/V de 2 à 16 V), car la droite ne passe pas exactement par 0. À 1 V, le frottement du moteur réduit la vitesse.

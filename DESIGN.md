# Dossier de Conception Technique

## Objectif
Détecter les deux drones les plus proches dans un essaim massif, avec allocation dynamique et navigation mémoire par arithmétique de pointeurs.

## Structure de données
- Structure hétérogène exigée :
  - `struct Drone { int id; float x; float y; float z; };`
- L'essaim est stocké dans un bloc contigu alloué dynamiquement via `malloc`.
- La traversée se fait par pointeurs (`p`, `p + k`, `*p`), sans indexation sur le tableau de drones.

## Architecture algorithmique
1. Génération/allocation de l'essaim dans un bloc mémoire unique.
2. Tri initial des drones par axe `x` (`qsort`).
3. Algorithme divide-and-conquer pour le plus proche voisin :
   - Division en deux sous-ensembles
   - Résolution récursive à gauche et à droite
   - Fusion via bande centrale (`strip`) autour de la médiane
   - Comparaison locale dans la bande triée par `y`
4. Retour du couple de drones minimisant la distance euclidienne 3D.

## Sécurité et robustesse
- Vérification systématique des allocations mémoire.
- Validation robuste des entrées utilisateur (`strtoul`, bornes, erreurs de conversion).
- Libération explicite des ressources allouées (`free`).
- Gestion d'erreurs avec messages explicites et codes de sortie appropriés.

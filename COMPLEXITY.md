# Audit Asymptotique et Preuve

## Référence naïve
Une double boucle complète compare chaque drone à tous les autres :
- Nombre de comparaisons : `N * (N - 1) / 2`
- Complexité : `O(N^2)`

Pour `N = 10000`, cette stratégie devient prohibitive en temps réel.

## Complexité de la solution retenue
La solution implémente un schéma divide-and-conquer avec tri initial.

1. Tri initial par `x` : `O(N log N)`
2. Récurrence principale :
   - 2 sous-problèmes de taille `N/2`
   - phase de fusion linéaire (construction de bande + tri local + voisinage borné)

On obtient la forme :
- `T(N) = 2T(N/2) + O(N log N)`
- donc `T(N) = O(N log^2 N)`

## Conclusion
La solution supprime le comportement quadratique de la force brute et passe à une complexité quasi-linéaire (log-linéaire), ce qui répond à l'objectif de réduction de saturation matérielle pour de grands essaims.

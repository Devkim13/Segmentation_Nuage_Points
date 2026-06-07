# Segmentation_Nuage_Points

## Description

Ce projet a pour objectif de traiter un nuage de points 3D afin de le transformer progressivement en une représentation structurée.

La chaîne actuelle du projet est :

```text
Nuage de points brut
→ Prétraitement
→ Calcul des normales
→ Segmentation par Region Growing
→ Coloration des régions
→ Construction d’un graphe d’adjacence
→ Export des résultats
```

À terme, le projet vise à construire un **graphe hiérarchique** où les nœuds représentent des objets ou primitives, et les arêtes représentent les relations spatiales entre ces objets.

---

## Objectifs actuels

Pour cette première version, le programme permet de :

- charger un fichier `.ply` ou `.pcd` ;
- réduire le nombre de points avec `VoxelGrid` ;
- calculer les normales du nuage ;
- segmenter le nuage avec une méthode de **Region Growing** ;
- colorer chaque région détectée ;
- construire un **Region Adjacency Graph** ;
- exporter les résultats ;
- mesurer le temps d’exécution de chaque étape.

---

## Technologies utilisées

- C++17
- CMake
- PCL
- CGAL

---

## Dépendances

Sous Ubuntu ou WSL :

```bash
sudo apt update
sudo apt install build-essential cmake libpcl-dev libcgal-dev
```

---

## Structure du projet

```text
Segmentation_Nuage_Points/
├── CMakeLists.txt
├── main.cpp
├── fragment.ply
└── build/
```

Après exécution, le dossier `build/` peut contenir :

```text
region_labels.txt
rag.json
regions_colored.ply
```

---

## Compilation

Depuis le dossier racine du projet :

```bash
mkdir -p build
cd build
cmake ..
make -j4
```

Après compilation, un exécutable nommé `seg` est généré dans le dossier `build`.

---

## Exécution

Si le fichier `fragment.ply` se trouve dans le dossier racine du projet :

```bash
./seg ../fragment.ply
```

Si le fichier `.ply` se trouve directement dans le dossier `build` :

```bash
./seg fragment.ply
```

Utilisation générale :

```bash
./seg chemin/vers/fichier.ply
```

---

## Pipeline implémenté

### 1. Chargement du nuage

Le programme charge un fichier passé en argument.

Formats supportés :

- `.ply`
- `.pcd`

Fonctions utilisées :

```cpp
pcl::io::loadPLYFile<PointT>(...)
pcl::io::loadPCDFile<PointT>(...)
```

---

### 2. Downsampling avec VoxelGrid

Le programme réduit le nombre de points avec `VoxelGrid`.

Objectifs :

- réduire le temps de calcul ;
- alléger le nuage ;
- conserver la structure géométrique globale.

Paramètre actuel :

```cpp
const double voxel_size = 0.05;
```

---

### 3. Calcul des normales

Les normales sont calculées avec PCL :

```cpp
pcl::NormalEstimation<PointT, pcl::Normal>
```

Le voisinage est recherché avec un `KdTree`.

Paramètre actuel :

```cpp
const double radius_normals = 0.2;
```

Les normales sont nécessaires pour comparer l’orientation locale des surfaces pendant la segmentation.

---

### 4. Segmentation par Region Growing

La segmentation actuelle est une version manuelle du **Region Growing**.

Principe :

1. choisir un point non encore labellisé ;
2. créer une nouvelle région ;
3. rechercher les voisins du point ;
4. comparer les normales ;
5. ajouter le voisin si l’angle entre les normales est inférieur au seuil ;
6. continuer jusqu’à stabilisation de la région.

Paramètres actuels :

```cpp
const double max_dist = 0.1;
const double max_angle_deg = 90.0;
const int min_region_size = 5;
```

Les petites régions sont rejetées et leurs points reçoivent le label `-1`.

---

### 5. Construction du RAG

Après la segmentation, le programme construit un **Region Adjacency Graph**.

Dans ce graphe :

- un nœud = une région segmentée ;
- une arête = une relation d’adjacence entre deux régions.

Le graphe est exporté dans :

```text
rag.json
```

Exemple :

```json
{
  "0": [1, 2],
  "1": [0, 3],
  "2": [0],
  "3": [1]
}
```

---

### 6. Coloration des régions

Chaque région reçoit une couleur différente.

Le résultat est exporté dans :

```text
regions_colored.ply
```

Les points non affectés à une région valide sont affichés en noir.

---

### 7. Export des labels

Le programme génère :

```text
region_labels.txt
```

Ce fichier contient un label par point.

Exemple :

```text
0
0
0
1
1
2
-1
3
```

Signification :

- `0`, `1`, `2`, etc. : identifiant de région ;
- `-1` : point non affecté ou région trop petite.

---

## Résultats générés

Après exécution, les fichiers suivants sont générés :

```text
regions_colored.ply
region_labels.txt
rag.json
```

### regions_colored.ply

Nuage de points segmenté et coloré.

Ouverture possible avec :

```bash
pcl_viewer regions_colored.ply
```

ou avec **CloudCompare**.

### region_labels.txt

Labels des régions pour chaque point.

### rag.json

Graphe d’adjacence entre les régions.

---

## Mesure du temps d’exécution

Le code contient un chronomètre pour mesurer chaque étape :

- chargement ;
- downsampling ;
- calcul des normales ;
- segmentation ;
- construction du RAG ;
- export ;
- temps total.

Exemple de sortie :

```text
[Temps] Chargement du nuage : 0.09 secondes
Points initiaux : 120000

[Temps] VoxelGrid / downsampling : 0.05 secondes
Après voxel : 45000 points

[Temps] Calcul des normales : 0.62 secondes

[Temps] Segmentation Region Growing : 2.10 secondes
Nombre de régions : 37

[Temps] Construction du RAG : 0.30 secondes
[Temps] Export des résultats : 0.05 secondes
[Temps] Temps total du programme : 3.21 secondes
```

Ces mesures permettent de comparer les performances et de justifier les choix techniques.

---

## Paramètres importants

```cpp
const double voxel_size = 0.05;
const double radius_normals = 0.2;
const double max_dist = 0.1;
const double max_angle_deg = 90.0;
const int min_region_size = 5;
```

| Paramètre | Rôle |
|---|---|
| `voxel_size` | Taille des voxels pour réduire le nuage |
| `radius_normals` | Rayon utilisé pour calculer les normales |
| `max_dist` | Rayon de recherche des voisins pour Region Growing |
| `max_angle_deg` | Angle maximal entre normales |
| `min_region_size` | Taille minimale d’une région valide |

---

## Limites actuelles

La version actuelle présente encore plusieurs limites :

- les paramètres sont fixés manuellement ;
- le prétraitement ne supprime pas encore explicitement les outliers statistiques ;
- l’extraction de primitives par RANSAC ou Hough n’est pas encore intégrée ;
- le graphe obtenu est un graphe d’adjacence, pas encore un graphe hiérarchique complet ;
- aucune métrique de type mIoU n’est encore calculée, car il n’y a pas encore de vérité terrain.

---

## Prochaines étapes

Les prochaines étapes prévues sont :

1. ajouter une suppression des outliers :
   - `StatisticalOutlierRemoval`
   - `RadiusOutlierRemoval`

2. comparer deux scénarios :
   - segmentation sans prétraitement ;
   - segmentation avec prétraitement.

3. ajouter l’extraction de primitives :
   - RANSAC pour détecter les plans ;
   - Hough pour certaines formes géométriques.

4. construire un graphe hiérarchique :
   - nœuds = régions ou primitives ;
   - arêtes = relations spatiales ;
   - hiérarchie = relations parent/enfant.

5. tester sur plusieurs fichiers :
   - `L001.ply`
   - `fandisk.obj`
   - `building.ply`

---

## État actuel

La version actuelle fournit une première base fonctionnelle :

```text
Chargement .ply/.pcd
→ VoxelGrid
→ Calcul des normales
→ Region Growing
→ Coloration des régions
→ Construction du RAG
→ Export des résultats
```

Cette base servira ensuite à intégrer l’extraction de primitives et la construction du graphe hiérarchique.

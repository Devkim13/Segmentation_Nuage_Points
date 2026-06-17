# Segmentation_Nuage_Points

## Description

Ce projet a pour objectif de transformer un nuage de points 3D en une représentation structurée sous forme de **graphe hiérarchique**.

La version précédente du projet réalisait principalement une segmentation par Region Growing et construisait un graphe d’adjacence entre régions. La nouvelle version évolue vers une approche plus proche de l’objectif final :

```text
Nuage de points brut
→ Prétraitement
→ Détection automatique de primitives géométriques
→ Classification géométrique approximative
→ Construction d’un graphe hiérarchique sous forme d’arbre
→ Export JSON et visualisation
```

L’idée est de représenter une scène intérieure, par exemple une chambre, une salle ou une classe, sous forme d’arbre :

```text
Scene / Pièce
├── Sol
│   └── Support horizontal / table
│       └── Objet posé
├── Mur 1
│   └── Objet mural / cadre / fenêtre
├── Mur 2
└── Plafond
```

Cette version ne fait pas encore de reconnaissance sémantique par deep learning. Elle utilise une **classification géométrique approximative** basée sur les plans, les hauteurs, les normales, les bounding boxes et les relations spatiales.

---

## Objectifs actuels

Le programme permet actuellement de :

- charger un fichier `.ply` ou `.pcd` ;
- nettoyer le nuage avec plusieurs filtres PCL ;
- réduire le nombre de points avec `VoxelGrid` ;
- supprimer les outliers statistiques avec `StatisticalOutlierRemoval` ;
- supprimer les points isolés avec `RadiusOutlierRemoval` ;
- extraire automatiquement les plans principaux avec RANSAC ;
- détecter des clusters dans les points restants ;
- classifier approximativement les primitives détectées ;
- construire une hiérarchie parent/enfant ;
- exporter la liste des primitives ;
- exporter un graphe hiérarchique sous forme d’arbre JSON ;
- exporter un nuage coloré par primitives ;
- mesurer le temps d’exécution de chaque étape.

---

## Technologies utilisées

- C++17
- CMake
- PCL
- CGAL
- Eigen

---

## Rôle de PCL et CGAL

### PCL

PCL est utilisé pour la partie traitement pratique du nuage de points :

- chargement des fichiers `.ply` et `.pcd` ;
- prétraitement avec `VoxelGrid`, `StatisticalOutlierRemoval` et `RadiusOutlierRemoval` ;
- extraction de plans avec RANSAC ;
- clustering euclidien des points restants ;
- calcul des centroïdes et bounding boxes ;
- visualisation du résultat ;
- export du nuage coloré.

### CGAL

CGAL est introduit dans la partie géométrique du projet. Dans cette version, il est utilisé pour calculer des distances robustes entre primitives grâce à `CGAL::squared_distance`.

Cela permet de commencer à intégrer CGAL dans la construction des relations spatiales du graphe hiérarchique.

À terme, CGAL pourra être utilisé davantage pour :

- des calculs géométriques robustes ;
- des distances point-plan ;
- la détection de primitives avec Efficient RANSAC ;
- l’analyse des relations spatiales entre objets.

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
├── box2.ply
└── build/
```

Après exécution, le dossier `build/` peut contenir :

```text
primitives_colored.ply
primitives.json
hierarchy.json
```

---

## Compilation

Depuis le dossier racine du projet :

```bash
rm -rf build
mkdir build
cd build
cmake ..
make -j4
```

Après compilation, un exécutable nommé `seg` est généré dans le dossier `build`.

---

## Exécution

Si le fichier `box2.ply` se trouve dans le dossier racine du projet :

```bash
./seg ../box2.ply
```

Utilisation générale :

```bash
./seg chemin/vers/fichier.ply
```

Formats supportés :

```text
.ply
.pcd
```

---

## Pipeline implémenté

### 1. Chargement du nuage

Le programme charge le fichier passé en argument.

Fonctions utilisées :

```cpp
pcl::io::loadPLYFile<PointT>(...)
pcl::io::loadPCDFile<PointT>(...)
```

Si le fichier n’est pas au format `.ply` ou `.pcd`, le programme affiche une erreur.

---

### 2. Prétraitement du nuage

Le prétraitement est composé de trois étapes.

#### 2.1 VoxelGrid

Le filtre `VoxelGrid` réduit le nombre de points.

Principe : l’espace est découpé en petits voxels, puis les points contenus dans chaque voxel sont remplacés par un point représentatif.

Objectifs :

- réduire le temps de calcul ;
- alléger le nuage ;
- conserver la structure globale de la scène.

Paramètre actuel :

```cpp
const double voxel_size = 0.05;
```

#### 2.2 StatisticalOutlierRemoval

Le filtre `StatisticalOutlierRemoval` supprime les points statistiquement aberrants.

Paramètres actuels :

```cpp
const int sor_mean_k = 30;
const double sor_stddev = 1.0;
```

#### 2.3 RadiusOutlierRemoval

Le filtre `RadiusOutlierRemoval` supprime les points isolés qui n’ont pas assez de voisins dans un rayon donné.

Paramètres actuels :

```cpp
const double ror_radius = 0.15;
const int ror_min_neighbors = 5;
```

---

### 3. Extraction des plans avec RANSAC

La fonction principale est :

```cpp
ExtractPlanesRANSAC(...)
```

Cette fonction détecte automatiquement les plans dominants du nuage.

Principe :

```text
chercher un plan dominant
→ extraire les points appartenant à ce plan
→ retirer ce plan du nuage
→ recommencer sur les points restants
```

Les plans détectés peuvent correspondre à :

- un sol ;
- un mur ;
- un plafond ;
- une table ;
- une face de boîte ;
- un cadre ;
- une fenêtre ;
- un tableau.

Paramètres actuels :

```cpp
const int max_planes = 12;
const int min_plane_points = 300;
const double plane_distance_threshold = 0.03;
```

Chaque plan détecté est stocké sous forme de `Primitive`.

---

### 4. Clustering des points restants

Après l’extraction des plans, certains points restent dans le nuage.

Ces points peuvent correspondre à :

- des objets non plans ;
- des petits objets ;
- des formes irrégulières ;
- des détails de scène ;
- du bruit restant.

La fonction utilisée est :

```cpp
ExtractClusters(...)
```

Elle applique un clustering euclidien sur les points restants.

Paramètres actuels :

```cpp
const double cluster_tolerance = 0.15;
const int min_cluster_size = 80;
const int max_cluster_size = 30000;
```

Chaque cluster détecté devient aussi une `Primitive`.

---

### 5. Structure `Primitive`

Chaque élément détecté est représenté par une structure `Primitive`.

Elle contient notamment :

- un identifiant `id` ;
- un nom `name` ;
- un type `type` ;
- un niveau dans l’arbre `level` ;
- un parent `parent_id` ;
- une relation avec le parent ;
- le nombre de points ;
- le centroïde ;
- la normale ;
- la bounding box ;
- les coefficients du plan si la primitive est plane.

Exemples de types possibles :

```text
floor
wall
ceiling
horizontal_support
object_on_support
wall_object
object_cluster
unknown
```

---

### 6. Classification géométrique approximative

La classification est réalisée par la fonction :

```cpp
ClassifyPlanes(...)
```

Elle utilise des règles géométriques simples.

#### Sol

```text
plan horizontal le plus bas
→ floor
```

#### Plafond

```text
plan horizontal le plus haut
et suffisamment éloigné du sol
→ ceiling
```

#### Mur

```text
plan vertical
→ wall
```

#### Support horizontal

```text
autre plan horizontal au-dessus du sol
→ horizontal_support
```

Un `horizontal_support` peut représenter approximativement :

- une table ;
- un bureau ;
- une étagère horizontale ;
- une surface de support.

Cette classification reste approximative. Elle prépare une future intégration du deep learning pour améliorer la reconnaissance sémantique.

---

### 7. Construction du graphe hiérarchique

La fonction principale est :

```cpp
BuildHierarchy(...)
```

Elle construit une structure parent/enfant.

Règles principales :

```text
Scene est la racine de l’arbre.
Floor, Wall et Ceiling sont enfants de Scene.
Les supports horizontaux sont rattachés au Floor.
Les objets au-dessus d’un support sont rattachés à ce support.
Les objets proches d’un mur sont rattachés au mur.
Les objets non classifiés sont rattachés au Floor ou à la Scene.
```

Exemple de hiérarchie attendue :

```text
Scene
├── Floor
│   └── Support_3
│       └── ObjectOnSupport_8
├── Wall_1
│   └── WallObject_9
├── Wall_2
└── Ceiling
```

---

## Résultats générés

Après exécution, le programme génère trois fichiers principaux.

```text
primitives_colored.ply
primitives.json
hierarchy.json
```

### primitives_colored.ply

Nuage de points coloré par primitives détectées.

Ouverture possible avec :

```bash
pcl_viewer primitives_colored.ply
```

ou avec CloudCompare.

### primitives.json

Liste plate de toutes les primitives détectées.

Exemple simplifié :

```json
{
  "primitives": [
    {
      "id": 1,
      "name": "Floor",
      "type": "floor",
      "level": 1,
      "parent_id": 0,
      "relation_to_parent": "part_of_scene",
      "number_of_points": 23514,
      "centroid": [0.1, 0.2, 0.0]
    }
  ]
}
```

### hierarchy.json

Arbre hiérarchique de la scène.

Exemple simplifié :

```json
{
  "id": 0,
  "name": "Scene",
  "type": "room_or_indoor_scene",
  "level": 0,
  "relation_to_parent": "root",
  "children": [
    {
      "id": 1,
      "name": "Floor",
      "type": "floor",
      "level": 1,
      "relation_to_parent": "part_of_scene",
      "children": []
    }
  ]
}
```

---

## Mesure du temps d’exécution

Le code contient un chronomètre pour mesurer chaque étape :

- chargement ;
- VoxelGrid ;
- suppression des outliers statistiques ;
- suppression des points isolés ;
- extraction des plans RANSAC ;
- clustering ;
- classification ;
- construction du graphe hiérarchique ;
- export ;
- temps total.

Exemple de sortie :

```text
[Temps] Chargement du nuage : 0.53 secondes
Points initiaux : 437126

[Temps] VoxelGrid / downsampling : 0.02 secondes
Après VoxelGrid : 38355 points

[Temps] StatisticalOutlierRemoval : 0.10 secondes
Après StatisticalOutlierRemoval : 30997 points

[Temps] RadiusOutlierRemoval : 0.04 secondes
Après RadiusOutlierRemoval : 28764 points

[Temps] Extraction des plans RANSAC : 0.30 secondes
Plan détecté 1 : 23514 points

[Temps] Clustering des objets restants : 0.05 secondes
[Temps] Classification géométrique des plans : 0.00 secondes
[Temps] Construction du graphe hiérarchique : 0.00 secondes
[Temps] Export des résultats : 0.01 secondes
[Temps] Temps total du programme : 2.00 secondes
```

Ces mesures permettent de comparer les performances et de justifier les choix techniques.

---

## Paramètres importants

```cpp
const double voxel_size = 0.05;

const int sor_mean_k = 30;
const double sor_stddev = 1.0;

const double ror_radius = 0.15;
const int ror_min_neighbors = 5;

const int max_planes = 12;
const int min_plane_points = 300;
const double plane_distance_threshold = 0.03;

const double cluster_tolerance = 0.15;
const int min_cluster_size = 80;
const int max_cluster_size = 30000;
```

| Paramètre | Rôle |
|---|---|
| `voxel_size` | Taille des voxels pour réduire le nuage |
| `sor_mean_k` | Nombre de voisins utilisés par StatisticalOutlierRemoval |
| `sor_stddev` | Seuil statistique de suppression des outliers |
| `ror_radius` | Rayon de recherche pour supprimer les points isolés |
| `ror_min_neighbors` | Nombre minimal de voisins dans le rayon |
| `max_planes` | Nombre maximal de plans à détecter |
| `min_plane_points` | Nombre minimal de points pour accepter un plan |
| `plane_distance_threshold` | Distance maximale d’un point au plan RANSAC |
| `cluster_tolerance` | Distance maximale entre deux points d’un même cluster |
| `min_cluster_size` | Taille minimale d’un cluster valide |
| `max_cluster_size` | Taille maximale d’un cluster valide |

---

## Différence avec l’ancienne version

Ancienne version :

```text
Nuage brut
→ Prétraitement
→ Calcul des normales
→ Region Growing
→ RAG
→ regions_colored.ply
→ rag.json
```

Nouvelle version :

```text
Nuage brut
→ Prétraitement complet
→ RANSAC plans
→ Clustering des restes
→ Classification géométrique
→ Graphe hiérarchique
→ primitives_colored.ply
→ primitives.json
→ hierarchy.json
```

La nouvelle version se rapproche davantage de l’objectif final du projet : obtenir une structure de scène interprétable, et pas seulement une segmentation colorée.

---

## Limites actuelles

La version actuelle reste une première version géométrique. Elle présente plusieurs limites :

- la classification est approximative ;
- les objets comme chaise, table, fenêtre ou cadre ne sont pas encore reconnus sémantiquement avec certitude ;
- la qualité dépend beaucoup des paramètres RANSAC et clustering ;
- les relations parent/enfant sont basées sur des règles géométriques simples ;
- le graphe hiérarchique peut nécessiter des ajustements selon le type de scène ;
- le deep learning n’est pas encore intégré ;
- CGAL est utilisé pour les distances géométriques, mais son module Efficient RANSAC n’est pas encore intégré.

---

## Prochaines étapes

Les prochaines étapes prévues sont :

1. améliorer la détection de primitives ;
2. tester le pipeline sur plusieurs scènes `.ply` ;
3. améliorer les règles de classification géométrique ;
4. mieux distinguer les murs, le sol, le plafond et les supports ;
5. améliorer les relations spatiales :
   - `posed_on` ;
   - `attached_to_wall` ;
   - `part_of_scene` ;
   - `near_to` ;
   - `inside` ;
6. intégrer plus fortement CGAL, notamment pour Efficient RANSAC ;
7. comparer cette approche géométrique avec des méthodes deep learning ;
8. étudier l’intégration du deep learning pour améliorer la classification sémantique.

---

## État actuel

La version actuelle fournit une première base fonctionnelle pour construire automatiquement un graphe hiérarchique à partir d’un nuage de points :

```text
Chargement .ply/.pcd
→ VoxelGrid
→ StatisticalOutlierRemoval
→ RadiusOutlierRemoval
→ Extraction de plans RANSAC
→ Clustering des points restants
→ Classification géométrique approximative
→ Construction de l’arbre hiérarchique
→ Export primitives.json
→ Export hierarchy.json
→ Export primitives_colored.ply
```

Cette version constitue la base géométrique du projet avant l’intégration future du deep learning.
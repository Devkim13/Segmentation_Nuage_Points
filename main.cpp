#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/extract_indices.h>

#include <pcl/features/normal_3d.h>

#include <pcl/search/kdtree.h>
#include <pcl/kdtree/kdtree_flann.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>

#include <pcl/common/common.h>
#include <pcl/common/centroid.h>

#include <pcl/visualization/pcl_visualizer.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Point_3.h>
#include <CGAL/squared_distance_3.h>

#include <Eigen/Dense>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <random>
#include <sstream>
#include <string>
#include <vector>

// ======================================================
// Types
// ======================================================

using PointT = pcl::PointXYZ;
using CloudT = pcl::PointCloud<PointT>;
using NormalCloudT = pcl::PointCloud<pcl::Normal>;

using Kernel = CGAL::Simple_cartesian<double>;
using CGALPoint = CGAL::Point_3<Kernel>;

// ======================================================
// Chronomètre
// ======================================================

class Timer
{
public:
    explicit Timer(const std::string& name)
        : name_(name),
          start_(std::chrono::high_resolution_clock::now())
    {
    }

    ~Timer()
    {
        auto end = std::chrono::high_resolution_clock::now();
        std::chrono::duration<double> elapsed = end - start_;

        std::cout << "[Temps] " << name_ << " : "
                  << elapsed.count() << " secondes" << std::endl;
    }

private:
    std::string name_;
    std::chrono::high_resolution_clock::time_point start_;
};

// ======================================================
// Structure Primitive
// Chaque primitive deviendra un noeud possible dans l'arbre.
// ======================================================

struct Primitive
{
    int id = -1;

    std::string name = "Unknown";
    std::string type = "unknown";

    int level = -1;
    int parent_id = -1;
    std::string relation_to_parent = "none";

    int number_of_points = 0;

    Eigen::Vector3f centroid = Eigen::Vector3f::Zero();
    Eigen::Vector3f normal = Eigen::Vector3f::Zero();

    float min_x = 0.0f;
    float max_x = 0.0f;
    float min_y = 0.0f;
    float max_y = 0.0f;
    float min_z = 0.0f;
    float max_z = 0.0f;

    // Coefficients du plan ax + by + cz + d = 0
    // Valable seulement pour les primitives planes.
    bool is_plane = false;
    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    float d = 0.0f;
};

// ======================================================
// Fonctions utilitaires
// ======================================================

std::string GetExtension(const std::string& filename)
{
    size_t pos = filename.find_last_of('.');

    if (pos == std::string::npos)
    {
        return "";
    }

    std::string ext = filename.substr(pos);

    std::transform(
        ext.begin(),
        ext.end(),
        ext.begin(),
        [](unsigned char c)
        {
            return static_cast<char>(std::tolower(c));
        }
    );

    return ext;
}

bool LoadPointCloud(const std::string& filename, CloudT::Ptr cloud)
{
    std::string ext = GetExtension(filename);

    int result = -1;

    if (ext == ".pcd")
    {
        result = pcl::io::loadPCDFile<PointT>(filename, *cloud);
    }
    else if (ext == ".ply")
    {
        result = pcl::io::loadPLYFile<PointT>(filename, *cloud);
    }
    else
    {
        std::cerr << "Format non supporté : " << ext << std::endl;
        std::cerr << "Utilise un fichier .ply ou .pcd" << std::endl;
        return false;
    }

    return result != -1 && !cloud->empty();
}

double DistanceCGAL(const Eigen::Vector3f& p1, const Eigen::Vector3f& p2)
{
    CGALPoint a(p1.x(), p1.y(), p1.z());
    CGALPoint b(p2.x(), p2.y(), p2.z());

    return std::sqrt(CGAL::to_double(CGAL::squared_distance(a, b)));
}

double DistancePointToPlane(const Primitive& plane, const Eigen::Vector3f& p)
{
    double numerator = std::abs(
        plane.a * p.x() +
        plane.b * p.y() +
        plane.c * p.z() +
        plane.d
    );

    double denominator = std::sqrt(
        plane.a * plane.a +
        plane.b * plane.b +
        plane.c * plane.c
    );

    if (denominator == 0.0)
    {
        return std::numeric_limits<double>::max();
    }

    return numerator / denominator;
}

bool OverlapXY(const Primitive& a, const Primitive& b)
{
    bool overlap_x = a.min_x <= b.max_x && a.max_x >= b.min_x;
    bool overlap_y = a.min_y <= b.max_y && a.max_y >= b.min_y;

    return overlap_x && overlap_y;
}

bool IsHorizontal(const Primitive& p)
{
    return std::abs(p.normal.z()) > 0.85f;
}

bool IsVertical(const Primitive& p)
{
    return std::abs(p.normal.z()) < 0.35f;
}

float AreaXY(const Primitive& p)
{
    return std::abs((p.max_x - p.min_x) * (p.max_y - p.min_y));
}

// ======================================================
// Prétraitement
// ======================================================

CloudT::Ptr DownsampleVoxelGrid(const CloudT::Ptr& cloud, double voxel_size)
{
    Timer timer("VoxelGrid / downsampling");

    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud);

    voxel.setLeafSize(
        static_cast<float>(voxel_size),
        static_cast<float>(voxel_size),
        static_cast<float>(voxel_size)
    );

    CloudT::Ptr cloud_downsampled(new CloudT);
    voxel.filter(*cloud_downsampled);

    return cloud_downsampled;
}

CloudT::Ptr RemoveStatisticalOutliers(
    const CloudT::Ptr& cloud,
    int mean_k,
    double stddev_mul_thresh
)
{
    Timer timer("StatisticalOutlierRemoval");

    pcl::StatisticalOutlierRemoval<PointT> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev_mul_thresh);

    CloudT::Ptr filtered(new CloudT);
    sor.filter(*filtered);

    return filtered;
}

CloudT::Ptr RemoveRadiusOutliers(
    const CloudT::Ptr& cloud,
    double radius,
    int min_neighbors
)
{
    Timer timer("RadiusOutlierRemoval");

    pcl::RadiusOutlierRemoval<PointT> ror;
    ror.setInputCloud(cloud);
    ror.setRadiusSearch(radius);
    ror.setMinNeighborsInRadius(min_neighbors);

    CloudT::Ptr filtered(new CloudT);
    ror.filter(*filtered);

    return filtered;
}

// ======================================================
// Calcul propriétés d'une primitive
// ======================================================

Primitive ComputePrimitiveProperties(
    int id,
    const std::string& name,
    const std::string& type,
    const CloudT::Ptr& cloud,
    bool is_plane,
    const pcl::ModelCoefficients::Ptr& coefficients = nullptr
)
{
    Primitive p;

    p.id = id;
    p.name = name;
    p.type = type;
    p.number_of_points = static_cast<int>(cloud->size());
    p.is_plane = is_plane;

    PointT min_pt;
    PointT max_pt;
    pcl::getMinMax3D(*cloud, min_pt, max_pt);

    p.min_x = min_pt.x;
    p.max_x = max_pt.x;
    p.min_y = min_pt.y;
    p.max_y = max_pt.y;
    p.min_z = min_pt.z;
    p.max_z = max_pt.z;

    Eigen::Vector4f centroid4;
    pcl::compute3DCentroid(*cloud, centroid4);

    p.centroid = Eigen::Vector3f(
        centroid4.x(),
        centroid4.y(),
        centroid4.z()
    );

    if (is_plane && coefficients && coefficients->values.size() >= 4)
    {
        p.a = coefficients->values[0];
        p.b = coefficients->values[1];
        p.c = coefficients->values[2];
        p.d = coefficients->values[3];

        Eigen::Vector3f n(p.a, p.b, p.c);

        if (n.norm() > 0.0f)
        {
            n.normalize();
        }

        // On force la normale à avoir une orientation stable.
        if (n.z() < 0.0f)
        {
            n = -n;
            p.a = -p.a;
            p.b = -p.b;
            p.c = -p.c;
            p.d = -p.d;
        }

        p.normal = n;
    }

    return p;
}

// ======================================================
// Extraction des plans par RANSAC
// ======================================================

std::vector<Primitive> ExtractPlanesRANSAC(
    CloudT::Ptr cloud,
    CloudT::Ptr remaining_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_primitives,
    int& next_id,
    int max_planes,
    int min_plane_points,
    double distance_threshold
)
{
    Timer timer("Extraction des plans RANSAC");

    std::vector<Primitive> planes;

    CloudT::Ptr current_cloud(new CloudT);
    *current_cloud = *cloud;

    std::mt19937 rng(10);
    std::uniform_int_distribution<int> color_dist(60, 255);

    for (int plane_index = 0; plane_index < max_planes; ++plane_index)
    {
        if (static_cast<int>(current_cloud->size()) < min_plane_points)
        {
            break;
        }

        pcl::SACSegmentation<PointT> seg;
        seg.setOptimizeCoefficients(true);
        seg.setModelType(pcl::SACMODEL_PLANE);
        seg.setMethodType(pcl::SAC_RANSAC);
        seg.setDistanceThreshold(distance_threshold);
        seg.setMaxIterations(1000);
        seg.setInputCloud(current_cloud);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);

        seg.segment(*inliers, *coefficients);

        if (static_cast<int>(inliers->indices.size()) < min_plane_points)
        {
            break;
        }

        pcl::ExtractIndices<PointT> extract;
        extract.setInputCloud(current_cloud);
        extract.setIndices(inliers);

        CloudT::Ptr plane_cloud(new CloudT);
        extract.setNegative(false);
        extract.filter(*plane_cloud);

        Primitive primitive = ComputePrimitiveProperties(
            next_id,
            "Plane_" + std::to_string(next_id),
            "plane_candidate",
            plane_cloud,
            true,
            coefficients
        );

        planes.push_back(primitive);

        unsigned char r = static_cast<unsigned char>(color_dist(rng));
        unsigned char g = static_cast<unsigned char>(color_dist(rng));
        unsigned char b = static_cast<unsigned char>(color_dist(rng));

        for (const auto& point : plane_cloud->points)
        {
            pcl::PointXYZRGB cp;
            cp.x = point.x;
            cp.y = point.y;
            cp.z = point.z;
            cp.r = r;
            cp.g = g;
            cp.b = b;
            colored_primitives->push_back(cp);
        }

        std::cout << "Plan détecté " << primitive.id
                  << " : " << primitive.number_of_points
                  << " points" << std::endl;

        next_id++;

        CloudT::Ptr new_remaining(new CloudT);
        extract.setNegative(true);
        extract.filter(*new_remaining);

        current_cloud = new_remaining;
    }

    *remaining_cloud = *current_cloud;

    return planes;
}

// ======================================================
// Clustering des points restants
// ======================================================

std::vector<Primitive> ExtractClusters(
    const CloudT::Ptr& remaining_cloud,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_primitives,
    int& next_id,
    double cluster_tolerance,
    int min_cluster_size,
    int max_cluster_size
)
{
    Timer timer("Clustering des objets restants");

    std::vector<Primitive> clusters;

    if (remaining_cloud->empty())
    {
        return clusters;
    }

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(remaining_cloud);

    std::vector<pcl::PointIndices> cluster_indices;

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance);
    ec.setMinClusterSize(min_cluster_size);
    ec.setMaxClusterSize(max_cluster_size);
    ec.setSearchMethod(tree);
    ec.setInputCloud(remaining_cloud);
    ec.extract(cluster_indices);

    std::mt19937 rng(20);
    std::uniform_int_distribution<int> color_dist(60, 255);

    for (const auto& indices : cluster_indices)
    {
        CloudT::Ptr cluster_cloud(new CloudT);

        for (int idx : indices.indices)
        {
            cluster_cloud->push_back(remaining_cloud->points[idx]);
        }

        Primitive primitive = ComputePrimitiveProperties(
            next_id,
            "Object_" + std::to_string(next_id),
            "object_cluster",
            cluster_cloud,
            false,
            nullptr
        );

        clusters.push_back(primitive);

        unsigned char r = static_cast<unsigned char>(color_dist(rng));
        unsigned char g = static_cast<unsigned char>(color_dist(rng));
        unsigned char b = static_cast<unsigned char>(color_dist(rng));

        for (const auto& point : cluster_cloud->points)
        {
            pcl::PointXYZRGB cp;
            cp.x = point.x;
            cp.y = point.y;
            cp.z = point.z;
            cp.r = r;
            cp.g = g;
            cp.b = b;
            colored_primitives->push_back(cp);
        }

        std::cout << "Cluster détecté " << primitive.id
                  << " : " << primitive.number_of_points
                  << " points" << std::endl;

        next_id++;
    }

    return clusters;
}

// ======================================================
// Classification géométrique approximative
// ======================================================

void ClassifyPlanes(std::vector<Primitive>& primitives)
{
    Timer timer("Classification géométrique des plans");

    std::vector<int> horizontal_ids;
    std::vector<int> vertical_ids;

    for (size_t i = 0; i < primitives.size(); ++i)
    {
        if (!primitives[i].is_plane)
        {
            continue;
        }

        if (IsHorizontal(primitives[i]))
        {
            horizontal_ids.push_back(static_cast<int>(i));
        }
        else if (IsVertical(primitives[i]))
        {
            vertical_ids.push_back(static_cast<int>(i));
        }
        else
        {
            primitives[i].type = "inclined_plane";
            primitives[i].name = "InclinedPlane_" + std::to_string(primitives[i].id);
        }
    }

    if (!horizontal_ids.empty())
    {
        int floor_idx = horizontal_ids[0];
        int ceiling_idx = horizontal_ids[0];

        for (int idx : horizontal_ids)
        {
            if (primitives[idx].centroid.z() < primitives[floor_idx].centroid.z())
            {
                floor_idx = idx;
            }

            if (primitives[idx].centroid.z() > primitives[ceiling_idx].centroid.z())
            {
                ceiling_idx = idx;
            }
        }

        primitives[floor_idx].type = "floor";
        primitives[floor_idx].name = "Floor";

        for (int idx : horizontal_ids)
        {
            if (idx == floor_idx)
            {
                continue;
            }

            float dz = std::abs(
                primitives[idx].centroid.z() -
                primitives[floor_idx].centroid.z()
            );

            if (idx == ceiling_idx && dz > 1.5f)
            {
                primitives[idx].type = "ceiling";
                primitives[idx].name = "Ceiling";
            }
            else
            {
                primitives[idx].type = "horizontal_support";
                primitives[idx].name = "Support_" + std::to_string(primitives[idx].id);
            }
        }
    }

    int wall_count = 1;

    for (int idx : vertical_ids)
    {
        primitives[idx].type = "wall";
        primitives[idx].name = "Wall_" + std::to_string(wall_count);
        wall_count++;
    }
}

// ======================================================
// Construction de l'arbre hiérarchique
// ======================================================

int FindFloorId(const std::vector<Primitive>& primitives)
{
    for (const auto& p : primitives)
    {
        if (p.type == "floor")
        {
            return p.id;
        }
    }

    return -1;
}

std::vector<int> FindWallIds(const std::vector<Primitive>& primitives)
{
    std::vector<int> walls;

    for (const auto& p : primitives)
    {
        if (p.type == "wall")
        {
            walls.push_back(p.id);
        }
    }

    return walls;
}

Primitive* FindPrimitiveById(std::vector<Primitive>& primitives, int id)
{
    for (auto& p : primitives)
    {
        if (p.id == id)
        {
            return &p;
        }
    }

    return nullptr;
}

const Primitive* FindPrimitiveByIdConst(const std::vector<Primitive>& primitives, int id)
{
    for (const auto& p : primitives)
    {
        if (p.id == id)
        {
            return &p;
        }
    }

    return nullptr;
}

void BuildHierarchy(std::vector<Primitive>& primitives)
{
    Timer timer("Construction du graphe hiérarchique");

    const int root_id = 0;

    int floor_id = FindFloorId(primitives);
    std::vector<int> wall_ids = FindWallIds(primitives);

    // Niveau 1 : sol, murs, plafond directement sous Scene.
    for (auto& p : primitives)
    {
        if (p.type == "floor" || p.type == "wall" || p.type == "ceiling")
        {
            p.parent_id = root_id;
            p.level = 1;
            p.relation_to_parent = "part_of_scene";
        }
    }

    // Supports horizontaux : table, bureau, plan intermédiaire.
    // Pour l'instant on les rattache au sol.
    for (auto& p : primitives)
    {
        if (p.type == "horizontal_support")
        {
            p.parent_id = (floor_id != -1) ? floor_id : root_id;
            p.level = (floor_id != -1) ? 2 : 1;
            p.relation_to_parent = "posed_on";
        }
    }

    // Plans inclinés ou inconnus : rattachés à la scène.
    for (auto& p : primitives)
    {
        if (p.type == "inclined_plane" || p.type == "plane_candidate")
        {
            p.parent_id = root_id;
            p.level = 1;
            p.relation_to_parent = "unknown_relation";
        }
    }

    // Clusters objets : chercher le meilleur support horizontal en dessous.
    for (auto& obj : primitives)
    {
        if (obj.type != "object_cluster")
        {
            continue;
        }

        int best_parent = -1;
        double best_score = std::numeric_limits<double>::max();

        for (const auto& support : primitives)
        {
            if (support.id == obj.id)
            {
                continue;
            }

            bool support_ok =
                support.type == "horizontal_support" ||
                support.type == "floor";

            if (!support_ok)
            {
                continue;
            }

            if (!OverlapXY(obj, support))
            {
                continue;
            }

            double vertical_gap = obj.min_z - support.max_z;

            if (vertical_gap >= -0.05 && vertical_gap < 0.35)
            {
                double center_distance = DistanceCGAL(obj.centroid, support.centroid);
                double score = std::abs(vertical_gap) + 0.01 * center_distance;

                if (score < best_score)
                {
                    best_score = score;
                    best_parent = support.id;
                }
            }
        }

        if (best_parent != -1)
        {
            obj.parent_id = best_parent;

            Primitive* parent = FindPrimitiveById(primitives, best_parent);
            obj.level = parent ? parent->level + 1 : 2;

            obj.relation_to_parent = "posed_on";
            obj.type = "object_on_support";
            obj.name = "ObjectOnSupport_" + std::to_string(obj.id);
        }
        else
        {
            // Si pas de support, tester si l'objet est proche d'un mur.
            int nearest_wall = -1;
            double best_wall_distance = std::numeric_limits<double>::max();

            for (int wall_id : wall_ids)
            {
                const Primitive* wall = FindPrimitiveByIdConst(primitives, wall_id);

                if (!wall)
                {
                    continue;
                }

                double distance = DistancePointToPlane(*wall, obj.centroid);

                if (distance < best_wall_distance)
                {
                    best_wall_distance = distance;
                    nearest_wall = wall_id;
                }
            }

            if (nearest_wall != -1 && best_wall_distance < 0.25)
            {
                obj.parent_id = nearest_wall;

                Primitive* parent = FindPrimitiveById(primitives, nearest_wall);
                obj.level = parent ? parent->level + 1 : 2;

                obj.relation_to_parent = "attached_to_wall";
                obj.type = "wall_object";
                obj.name = "WallObject_" + std::to_string(obj.id);
            }
            else
            {
                obj.parent_id = (floor_id != -1) ? floor_id : root_id;

                Primitive* parent = FindPrimitiveById(primitives, obj.parent_id);
                obj.level = parent ? parent->level + 1 : 1;

                obj.relation_to_parent = "unknown_or_on_floor";
                obj.name = "UnclassifiedObject_" + std::to_string(obj.id);
            }
        }
    }
}

// ======================================================
// Exports JSON
// ======================================================

void ExportPrimitivesJson(
    const std::vector<Primitive>& primitives,
    const std::string& filename
)
{
    std::ofstream f(filename);

    f << "{\n";
    f << "  \"primitives\": [\n";

    for (size_t i = 0; i < primitives.size(); ++i)
    {
        const auto& p = primitives[i];

        f << "    {\n";
        f << "      \"id\": " << p.id << ",\n";
        f << "      \"name\": \"" << p.name << "\",\n";
        f << "      \"type\": \"" << p.type << "\",\n";
        f << "      \"level\": " << p.level << ",\n";
        f << "      \"parent_id\": " << p.parent_id << ",\n";
        f << "      \"relation_to_parent\": \"" << p.relation_to_parent << "\",\n";
        f << "      \"number_of_points\": " << p.number_of_points << ",\n";
        f << "      \"centroid\": ["
          << p.centroid.x() << ", "
          << p.centroid.y() << ", "
          << p.centroid.z() << "],\n";
        f << "      \"bbox\": {\n";
        f << "        \"min\": [" << p.min_x << ", " << p.min_y << ", " << p.min_z << "],\n";
        f << "        \"max\": [" << p.max_x << ", " << p.max_y << ", " << p.max_z << "]\n";
        f << "      }\n";
        f << "    }";

        if (i + 1 < primitives.size())
        {
            f << ",";
        }

        f << "\n";
    }

    f << "  ]\n";
    f << "}\n";
}

void WriteNodeRecursive(
    std::ofstream& f,
    int node_id,
    const std::string& name,
    const std::string& type,
    int level,
    const std::string& relation,
    const std::vector<Primitive>& primitives,
    int indent
)
{
    std::string sp(indent, ' ');

    f << sp << "{\n";
    f << sp << "  \"id\": " << node_id << ",\n";
    f << sp << "  \"name\": \"" << name << "\",\n";
    f << sp << "  \"type\": \"" << type << "\",\n";
    f << sp << "  \"level\": " << level << ",\n";
    f << sp << "  \"relation_to_parent\": \"" << relation << "\",\n";
    f << sp << "  \"children\": [\n";

    std::vector<const Primitive*> children;

    for (const auto& p : primitives)
    {
        if (p.parent_id == node_id)
        {
            children.push_back(&p);
        }
    }

    for (size_t i = 0; i < children.size(); ++i)
    {
        const Primitive* child = children[i];

        WriteNodeRecursive(
            f,
            child->id,
            child->name,
            child->type,
            child->level,
            child->relation_to_parent,
            primitives,
            indent + 4
        );

        if (i + 1 < children.size())
        {
            f << ",";
        }

        f << "\n";
    }

    f << sp << "  ]\n";
    f << sp << "}";
}

void ExportHierarchyJson(
    const std::vector<Primitive>& primitives,
    const std::string& filename
)
{
    std::ofstream f(filename);

    WriteNodeRecursive(
        f,
        0,
        "Scene",
        "room_or_indoor_scene",
        0,
        "root",
        primitives,
        0
    );

    f << "\n";
}

// ======================================================
// Affichage console
// ======================================================

void PrintPrimitives(const std::vector<Primitive>& primitives)
{
    std::cout << "\n===== Primitives détectées =====" << std::endl;

    for (const auto& p : primitives)
    {
        std::cout << "ID " << p.id
                  << " | " << p.name
                  << " | type=" << p.type
                  << " | points=" << p.number_of_points
                  << " | level=" << p.level
                  << " | parent=" << p.parent_id
                  << " | relation=" << p.relation_to_parent
                  << std::endl;
    }

    std::cout << "================================\n" << std::endl;
}

// ======================================================
// Visualisation
// ======================================================

void VisualizeColoredCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud
)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Primitives + Graphe hierarchique")
    );

    viewer->setBackgroundColor(0.0, 0.0, 0.0);

    viewer->addPointCloud<pcl::PointXYZRGB>(
        colored_cloud,
        "primitives"
    );

    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        3,
        "primitives"
    );

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
}

// ======================================================
// Main
// ======================================================

int main(int argc, char** argv)
{
    Timer total_timer("Temps total du programme");

    if (argc < 2)
    {
        std::cout << "Usage : ./seg cloud.ply" << std::endl;
        return 1;
    }

    std::string filename = argv[1];

    // ==================================================
    // Paramètres principaux
    // ==================================================

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

    // ==================================================
    // 1. Chargement
    // ==================================================

    CloudT::Ptr cloud(new CloudT);

    {
        Timer timer("Chargement du nuage");

        if (!LoadPointCloud(filename, cloud))
        {
            std::cout << "Erreur : impossible de charger "
                      << filename << std::endl;
            return 1;
        }
    }

    std::cout << "Points initiaux : " << cloud->size() << std::endl;

    // ==================================================
    // 2. Prétraitement
    // ==================================================

    cloud = DownsampleVoxelGrid(cloud, voxel_size);
    std::cout << "Après VoxelGrid : " << cloud->size() << " points" << std::endl;

    cloud = RemoveStatisticalOutliers(cloud, sor_mean_k, sor_stddev);
    std::cout << "Après StatisticalOutlierRemoval : "
              << cloud->size() << " points" << std::endl;

    cloud = RemoveRadiusOutliers(cloud, ror_radius, ror_min_neighbors);
    std::cout << "Après RadiusOutlierRemoval : "
              << cloud->size() << " points" << std::endl;

    // ==================================================
    // 3. Extraction des plans principaux
    // ==================================================

    int next_id = 1;

    CloudT::Ptr remaining_cloud(new CloudT);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_primitives(
        new pcl::PointCloud<pcl::PointXYZRGB>
    );

    std::vector<Primitive> primitives = ExtractPlanesRANSAC(
        cloud,
        remaining_cloud,
        colored_primitives,
        next_id,
        max_planes,
        min_plane_points,
        plane_distance_threshold
    );

    std::cout << "Points restants après extraction des plans : "
              << remaining_cloud->size() << std::endl;

    // ==================================================
    // 4. Clustering des points restants
    // ==================================================

    std::vector<Primitive> clusters = ExtractClusters(
        remaining_cloud,
        colored_primitives,
        next_id,
        cluster_tolerance,
        min_cluster_size,
        max_cluster_size
    );

    primitives.insert(
        primitives.end(),
        clusters.begin(),
        clusters.end()
    );

    // ==================================================
    // 5. Classification géométrique
    // ==================================================

    ClassifyPlanes(primitives);

    // ==================================================
    // 6. Construction de l'arbre hiérarchique
    // ==================================================

    BuildHierarchy(primitives);

    // ==================================================
    // 7. Affichage console
    // ==================================================

    PrintPrimitives(primitives);

    // ==================================================
    // 8. Exports
    // ==================================================

    {
        Timer timer("Export des résultats");

        ExportPrimitivesJson(primitives, "primitives.json");
        ExportHierarchyJson(primitives, "hierarchy.json");

        colored_primitives->width = static_cast<uint32_t>(colored_primitives->size());
        colored_primitives->height = 1;
        colored_primitives->is_dense = true;

        pcl::io::savePLYFileBinary("primitives_colored.ply", *colored_primitives);
    }

    std::cout << "Exports générés :" << std::endl;
    std::cout << " - primitives.json" << std::endl;
    std::cout << " - hierarchy.json" << std::endl;
    std::cout << " - primitives_colored.ply" << std::endl;

    // ==================================================
    // 9. Visualisation
    // ==================================================

    VisualizeColoredCloud(colored_primitives);

    return 0;
}
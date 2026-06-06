#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/features/normal_3d.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/kdtree.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <CGAL/Simple_cartesian.h>
#include <CGAL/Point_3.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <map>
#include <queue>
#include <random>
#include <set>
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
// Chronomètre simple
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

double AngleBetweenNormals(const pcl::Normal& n1, const pcl::Normal& n2)
{
    double x1 = n1.normal_x;
    double y1 = n1.normal_y;
    double z1 = n1.normal_z;

    double x2 = n2.normal_x;
    double y2 = n2.normal_y;
    double z2 = n2.normal_z;

    if (!std::isfinite(x1) || !std::isfinite(y1) || !std::isfinite(z1) ||
        !std::isfinite(x2) || !std::isfinite(y2) || !std::isfinite(z2))
    {
        return 180.0;
    }

    double dot = x1 * x2 + y1 * y2 + z1 * z2;

    double norm1 = std::sqrt(x1 * x1 + y1 * y1 + z1 * z1);
    double norm2 = std::sqrt(x2 * x2 + y2 * y2 + z2 * z2);

    if (norm1 == 0.0 || norm2 == 0.0)
    {
        return 180.0;
    }

    double cosang = dot / (norm1 * norm2);

    // Rend la comparaison indépendante du sens de la normale.
    cosang = std::abs(cosang);

    cosang = std::max(-1.0, std::min(1.0, cosang));

    constexpr double PI = 3.14159265358979323846;

    return std::acos(cosang) * 180.0 / PI;
}

// ======================================================
// Prétraitement : VoxelGrid
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

// ======================================================
// Estimation des normales
// ======================================================

NormalCloudT::Ptr EstimateNormals(const CloudT::Ptr& cloud, double radius_normals)
{
    Timer timer("Calcul des normales");

    NormalCloudT::Ptr normals(new NormalCloudT);

    pcl::NormalEstimation<PointT, pcl::Normal> normal_estimation;
    normal_estimation.setInputCloud(cloud);

    pcl::search::KdTree<PointT>::Ptr normal_tree(new pcl::search::KdTree<PointT>);
    normal_estimation.setSearchMethod(normal_tree);
    normal_estimation.setRadiusSearch(radius_normals);

    normal_estimation.compute(*normals);

    return normals;
}

// ======================================================
// Region Growing manuel
// ======================================================

std::vector<int> RegionGrowingSegmentation(
    const CloudT::Ptr& cloud,
    const NormalCloudT::Ptr& normals,
    double max_dist,
    double max_angle_deg,
    int min_region_size,
    int& n_regions
)
{
    Timer timer("Segmentation Region Growing");

    const size_t n_points = cloud->size();

    std::vector<int> labels(n_points, -1);
    int current_label = 0;

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);

    for (size_t seed_idx = 0; seed_idx < n_points; ++seed_idx)
    {
        if (labels[seed_idx] != -1)
        {
            continue;
        }

        std::queue<int> q;
        std::vector<int> region_points;

        labels[seed_idx] = current_label;
        q.push(static_cast<int>(seed_idx));
        region_points.push_back(static_cast<int>(seed_idx));

        while (!q.empty())
        {
            int idx = q.front();
            q.pop();

            std::vector<int> neighbor_indices;
            std::vector<float> neighbor_distances;

            int nb = kdtree.radiusSearch(
                cloud->points[idx],
                max_dist,
                neighbor_indices,
                neighbor_distances
            );

            for (int k = 0; k < nb; ++k)
            {
                int j = neighbor_indices[k];

                if (labels[j] != -1)
                {
                    continue;
                }

                double angle = AngleBetweenNormals(
                    normals->points[idx],
                    normals->points[j]
                );

                if (angle < max_angle_deg)
                {
                    labels[j] = current_label;
                    q.push(j);
                    region_points.push_back(j);
                }
            }
        }

        if (static_cast<int>(region_points.size()) < min_region_size)
        {
            for (int idx : region_points)
            {
                labels[idx] = -1;
            }
        }
        else
        {
            std::cout << "Région " << current_label
                      << " : " << region_points.size()
                      << " points" << std::endl;

            current_label++;
        }
    }

    n_regions = current_label;

    return labels;
}

// ======================================================
// Construction du RAG : Region Adjacency Graph
// ======================================================

std::map<int, std::set<int>> BuildRAG(
    const CloudT::Ptr& cloud,
    const std::vector<int>& labels,
    int n_regions,
    double max_dist
)
{
    Timer timer("Construction du RAG");

    std::map<int, std::set<int>> rag_adj;

    for (int r = 0; r < n_regions; ++r)
    {
        rag_adj[r] = {};
    }

    pcl::KdTreeFLANN<PointT> kdtree;
    kdtree.setInputCloud(cloud);

    for (size_t i = 0; i < cloud->size(); ++i)
    {
        if (labels[i] < 0)
        {
            continue;
        }

        std::vector<int> neighbor_indices;
        std::vector<float> neighbor_distances;

        int nb = kdtree.radiusSearch(
            cloud->points[i],
            max_dist,
            neighbor_indices,
            neighbor_distances
        );

        for (int k = 0; k < nb; ++k)
        {
            int j = neighbor_indices[k];

            if (labels[j] < 0)
            {
                continue;
            }

            if (labels[j] != labels[i])
            {
                int a = labels[i];
                int b = labels[j];

                rag_adj[a].insert(b);
                rag_adj[b].insert(a);
            }
        }
    }

    return rag_adj;
}

// ======================================================
// Affichage console du RAG
// ======================================================

void PrintRAG(const std::map<int, std::set<int>>& rag_adj, int max_lines = 10)
{
    std::cout << "Extrait du RAG :" << std::endl;

    int printed = 0;

    for (const auto& kv : rag_adj)
    {
        std::cout << "Région " << kv.first << " adjacente à : ";

        for (int neigh : kv.second)
        {
            std::cout << neigh << " ";
        }

        std::cout << std::endl;

        printed++;

        if (printed >= max_lines)
        {
            break;
        }
    }
}

// ======================================================
// Coloration des régions
// ======================================================

pcl::PointCloud<pcl::PointXYZRGB>::Ptr ColorRegions(
    const CloudT::Ptr& cloud,
    const std::vector<int>& labels,
    int n_regions
)
{
    Timer timer("Coloration des régions");

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(
        new pcl::PointCloud<pcl::PointXYZRGB>
    );

    const size_t n_points = cloud->size();

    colored_cloud->resize(n_points);
    colored_cloud->width = static_cast<uint32_t>(n_points);
    colored_cloud->height = 1;
    colored_cloud->is_dense = cloud->is_dense;

    std::mt19937 rng(0);
    std::uniform_int_distribution<int> color_dist(50, 255);

    std::vector<std::array<unsigned char, 3>> region_colors(n_regions);

    for (int r = 0; r < n_regions; ++r)
    {
        region_colors[r] = {
            static_cast<unsigned char>(color_dist(rng)),
            static_cast<unsigned char>(color_dist(rng)),
            static_cast<unsigned char>(color_dist(rng))
        };
    }

    for (size_t i = 0; i < n_points; ++i)
    {
        pcl::PointXYZRGB p;

        p.x = cloud->points[i].x;
        p.y = cloud->points[i].y;
        p.z = cloud->points[i].z;

        int lab = labels[i];

        if (lab >= 0 && lab < n_regions)
        {
            p.r = region_colors[lab][0];
            p.g = region_colors[lab][1];
            p.b = region_colors[lab][2];
        }
        else
        {
            p.r = 0;
            p.g = 0;
            p.b = 0;
        }

        colored_cloud->points[i] = p;
    }

    return colored_cloud;
}

// ======================================================
// Exports
// ======================================================

void ExportLabels(const std::vector<int>& labels, const std::string& filename)
{
    std::ofstream f(filename);

    for (int label : labels)
    {
        f << label << "\n";
    }
}

void ExportRAGJson(
    const std::map<int, std::set<int>>& rag_adj,
    const std::string& filename
)
{
    std::ofstream f(filename);

    f << "{\n";

    bool first_node = true;

    for (const auto& kv : rag_adj)
    {
        if (!first_node)
        {
            f << ",\n";
        }

        first_node = false;

        f << "  \"" << kv.first << "\": [";

        bool first_neighbor = true;

        for (int neigh : kv.second)
        {
            if (!first_neighbor)
            {
                f << ", ";
            }

            first_neighbor = false;
            f << neigh;
        }

        f << "]";
    }

    f << "\n}\n";
}

// ======================================================
// Visualisation
// ======================================================

void VisualizeColoredCloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& colored_cloud
)
{
    pcl::visualization::PCLVisualizer::Ptr viewer(
        new pcl::visualization::PCLVisualizer("Region Growing + RAG")
    );

    viewer->setBackgroundColor(0.0, 0.0, 0.0);

    viewer->addPointCloud<pcl::PointXYZRGB>(
        colored_cloud,
        "regions"
    );

    viewer->setPointCloudRenderingProperties(
        pcl::visualization::PCL_VISUALIZER_POINT_SIZE,
        3,
        "regions"
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

    // Paramètres principaux
    const double voxel_size = 0.05;
    const double radius_normals = 0.2;
    const double max_dist = 0.1;
    const double max_angle_deg = 90.0;
    const int min_region_size = 5;

    // 1. Chargement du nuage
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

    // 2. Downsampling
    CloudT::Ptr cloud_downsampled = DownsampleVoxelGrid(cloud, voxel_size);
    cloud = cloud_downsampled;

    std::cout << "Après voxel : " << cloud->size() << " points" << std::endl;

    // 3. Calcul des normales
    NormalCloudT::Ptr normals = EstimateNormals(cloud, radius_normals);

    if (normals->size() != cloud->size())
    {
        std::cout << "Erreur : nombre de normales incorrect." << std::endl;
        return 1;
    }

    // 4. Region Growing
    int n_regions = 0;

    std::vector<int> labels = RegionGrowingSegmentation(
        cloud,
        normals,
        max_dist,
        max_angle_deg,
        min_region_size,
        n_regions
    );

    std::cout << "Nombre de régions : " << n_regions << std::endl;

    // 5. Construction du RAG
    std::map<int, std::set<int>> rag_adj = BuildRAG(
        cloud,
        labels,
        n_regions,
        max_dist
    );

    PrintRAG(rag_adj);

    // 6. Coloration
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud =
        ColorRegions(cloud, labels, n_regions);

    // 7. Exports
    {
        Timer timer("Export des résultats");

        ExportLabels(labels, "region_labels.txt");
        ExportRAGJson(rag_adj, "rag.json");
        pcl::io::savePLYFileBinary("regions_colored.ply", *colored_cloud);
    }

    std::cout << "Exports générés :" << std::endl;
    std::cout << " - region_labels.txt" << std::endl;
    std::cout << " - rag.json" << std::endl;
    std::cout << " - regions_colored.ply" << std::endl;

    // 8. Visualisation
    VisualizeColoredCloud(colored_cloud);

    return 0;
}
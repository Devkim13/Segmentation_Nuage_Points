#ifndef PRIMITIVE_H
#define PRIMITIVE_H

#include <Eigen/Dense>
#include <string>

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

    bool is_plane = false;

    float a = 0.0f;
    float b = 0.0f;
    float c = 0.0f;
    float d = 0.0f;
};

#endif
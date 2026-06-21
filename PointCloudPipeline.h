#ifndef POINTCLOUDPIPELINE_H
#define POINTCLOUDPIPELINE_H

#include <string>

struct PipelineResult
{
    bool success = false;
    std::string logs;

    std::string raw_cloud_ply = "raw_cloud.ply";
    std::string preprocessed_cloud_ply = "preprocessed_cloud.ply";
    std::string primitives_json = "primitives.json";
    std::string hierarchy_json = "hierarchy.json";
    std::string primitives_colored_ply = "primitives_colored.ply";
};

class PointCloudPipeline
{
public:
    PipelineResult run(const std::string& filename);

    static void visualizeRawAndPreprocessed(
        const std::string& raw_file,
        const std::string& preprocessed_file
    );
};

#endif
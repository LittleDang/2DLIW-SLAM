#pragma once
#include "utilies/macro.h"
#include <Eigen/Dense>
#include <map>
#include <memory>
#include <tuple>
#include <vector>
namespace lvio_2d
{
    struct feature_map_point
    {
        Eigen::Vector3d world_point;
        bool has_frozen;
        feature_map_point() {}
        feature_map_point(const Eigen::Vector3d &p, const bool &frozen) : has_frozen(frozen), world_point(p) {}
    };
    struct feature_map
    {
        using ptr = std::shared_ptr<feature_map>;
        std::map<long long, feature_map_point> world_points;
    };
} // namespace lvio_2d

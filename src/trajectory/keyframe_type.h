#pragma once
#include "trajectory/laser_type.h"
#include "utilies/macro.h"
#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <stdint.h>
#include <tuple>
#include <vector>
namespace lvio_2d
{
    struct edge
    {
        using ptr = std::shared_ptr<edge>;
        int index1;
        int index2;
        int times;
        Eigen::Isometry3d tf12;
        Eigen::Matrix<double, 6, 6> J;
        edge(const int &index1_, const int &index2_, const Eigen::Isometry3d &tf12_)
        {
            index1 = index1_;
            index2 = index2_;
            tf12 = tf12_;
            times = 0;
            J = Eigen::Matrix<double, 6, 6>::Identity();
        }
        void set_J(Eigen::Matrix<double, 6, 6> &J_)
        {
            J = J_;
        }
    };
    struct des_j
    {
        double aij;
        int dij;
        int j;
        des_j(double aij_, int dij_, int j_)
        {
            aij = aij_;
            dij = dij_;
            j = j_;
        }
    };
    struct des_i
    {
        int i;
        std::vector<std::uint64_t> quick_des;
        std::vector<des_j> des_ij;
        des_i();
        void set_1(int dij);
    };
    struct laser_match_index
    {
        using ptr = std::shared_ptr<laser_match_index>;
        std::vector<int> p1;
        std::vector<int> p2;
    };
    struct laser_match_point
    {
        using ptr = std::shared_ptr<laser_match_point>;
        Eigen::Isometry3d tf1;
        Eigen::Isometry3d tf2;
        scan::ptr scan1;
        scan::ptr scan2;
        int index1;
        int index2;
        laser_match::ptr laser_match_ptr;
        Eigen::Isometry3d tf12;
        std::vector<Eigen::Vector3d> p1;
        std::vector<Eigen::Vector3d> p2;
    };
    struct laser_map_feature
    {
        using ptr = std::shared_ptr<laser_map_feature>;
        std::vector<Eigen::Vector3d> points;
        std::vector<scan::ptr> scans;
        std::vector<Eigen::Isometry3d> tfs;
        std::vector<des_i> dess;
        long long index;
        Eigen::Isometry3d origin;
        laser_map_feature(const std::vector<std::vector<Eigen::Vector3d>> &concers,
                          const std::vector<scan::ptr> &scans_,
                          const std::vector<Eigen::Isometry3d> &tfs_,
                          const long long &index_,
                          const Eigen::Isometry3d &origin_);
        static laser_match_index::ptr match_des(const des_i &d1, const des_i &d2);
        static laser_match_point::ptr match_map(laser_map_feature::ptr m1, laser_map_feature::ptr m2);
    };
} // namespace lvio_2d
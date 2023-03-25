#pragma once
#include "timerAndColor/timer.h"
#include "utilies/macro.h"
#include "utilies/my_struct.h"
#include <Eigen/Dense>
#include <deque>
#include <memory>
#include <tuple>
#include <vector>
namespace lvio_2d
{

    struct line
    {
        using ptr = std::shared_ptr<line>;
        Eigen::Vector3d p1;
        Eigen::Vector3d p2;
        Eigen::Vector3d abc;
        double len;
        line(const Eigen::Vector3d &p1_, const Eigen::Vector3d &p2_, const Eigen::Vector3d &abc_);
    };
    struct scan
    {
        using ptr = std::shared_ptr<scan>;
        double time;
        int w;
        int h;
        double resolution;
        my_2d_vec<std::vector<line::ptr>> line_map;
        // std::vector<std::vector<std::vector<line::ptr>>> line_map;
        std::vector<line::ptr> lines;
        std::vector<Eigen::Vector3d> concers;
        std::vector<Eigen::Vector3d> points;
        bool is_index_valid(const int &r, const int &c)
        {
            return r >= 0 && r < h && c >= 0 && c < w;
        }
        std::tuple<int, int> xy_to_index(const double &x, const double &y)
        {
            return {x / resolution + w / 2, y / resolution + h / 2};
        }
        scan(const int &w_, const int &h_, const double &resolution_, const double &time_) : line_map(h_, w_)
        {
            // lmicroTimer("scan construct");
            w = w_;
            h = h_;
            resolution = resolution_;
            time = time_;
            // line_map = std::vector<std::vector<std::vector<line::ptr>>>(h_, std::vector<std::vector<line::ptr>>(
            //                                                                  w_));
        }
        void add_line(const std::vector<Eigen::Vector3d> &points, const int &index1, const int &index2, bool add_concers = true);

        void add_line(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, bool add_concers);
    };
    struct laser_submap
    {
        using ptr = std::shared_ptr<laser_submap>;
        Eigen::Vector3d current_p;
        Eigen::Vector3d current_q;
        scan::ptr scan_ptr;
        laser_submap(const scan::ptr &scan_ptr_,
                     const Eigen::Vector3d &current_p_,
                     const Eigen::Vector3d &current_q_)
        {
            current_p = current_p_;
            current_q = current_q_;
            scan_ptr = scan_ptr_;
        }
    };
    struct laser_map
    {
        using ptr = std::shared_ptr<laser_map>;
        std::deque<laser_submap::ptr> submaps;
    };
    struct laser_match
    {
        using ptr = std::shared_ptr<laser_match>;

        std::vector<line::ptr> lines1;
        std::vector<line::ptr> lines2;

        Eigen::Vector3d p1, q1, p2, q2;
        scan::ptr scan2;
    };
} // namespace lvio_2d

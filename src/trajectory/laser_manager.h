#pragma once
#include "trajectory/laser_type.h"
#include "trajectory/sensor.h"
#include "utilies/utilies.h"
namespace lvio_2d
{

    class laser_manager
    {
    public:
        laser_manager();

        scan::ptr spawn_scan(const sensor::laser::u_ptr &laser);
        void add_scan(const scan::ptr &scan_ptr,
                      const Eigen::Vector3d &current_p,
                      const Eigen::Vector3d &current_q);
        laser_match::ptr match_with_front(const scan::ptr scan_ptr, const Eigen::Vector3d &current_p,
                                          const Eigen::Vector3d &current_q);
        laser_match::ptr match_with_back(const scan::ptr scan_ptr, const Eigen::Vector3d &current_p,
                                         const Eigen::Vector3d &current_q);

        laser_match::ptr match_with_ref(const scan::ptr scan_ptr, const Eigen::Vector3d &current_p,
                                        const Eigen::Vector3d &current_q);

        laser_submap::ptr pop_scan();

        void clear_all_scan();

        std::deque<laser_submap::ptr> &get_keyframs()
        {
            return key_frame;
        }
        static laser_match::ptr do_match(const scan::ptr &scan1, const scan::ptr &scan2,
                                         const Eigen::Vector3d &p1, const Eigen::Vector3d &q1,
                                         const Eigen::Vector3d &p2, const Eigen::Vector3d &q2,
                                         const int kk = 0);

    private:
        int w;
        int h;
        double resolution;
        double line_max_tolerance_angle;

        std::deque<laser_submap::ptr> key_frame;

        laser_submap::ptr ref_submap_ptr;
        laser_submap::ptr spawnning_ref_submap_ptr;
        Eigen::Isometry3d last_add_tf;
        int current_count;
    };
} // namespace lvio_2d
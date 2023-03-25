#pragma once
#include "factor/camera_factor.h"
#include "factor/ground_factor.h"
#include "factor/imu_factor.h"
#include "factor/laser_factor.h"
#include "factor/marginalization_factor.h"
#include "factor/wheel_factor.h"
#include "trajectory/camera_manager.h"
#include "trajectory/trajectory_type.h"

namespace lvio_2d
{
    struct status_bolck_index
    {
        // X
        int p;
        int q;
        int v;
        int bs;

        // r
        int laser_res_index;
        int imu_res_index;
        int wheel_res_index;
        int ground_res_index;
    };

    class solver
    {
    private:
        bool has_linearized_block;
        // lastest frame: p q vbs m
        //
        // world point :at lastest camera point
        Eigen::VectorXd linearized_X;
        Eigen::MatrixXd linearized_jacobians;
        Eigen::VectorXd linearized_residuals;
        std::vector<long long> linearized_world_ids;
        std::vector<Eigen::Vector3d> linearized_world_points;

        // J R
        Eigen::MatrixXd J;
        Eigen::MatrixXd R;

        // index info
        std::vector<status_bolck_index> all_status_block_indexs;
        std::map<long long, int> world_point_indexs;
        int camera_res_index;
        int lastest_wolrd_point_index;

        // ext camera
        Eigen::Vector3d ground_v1, ground_v2, ground_v3;
        double v1, v2, v3;
        void clac_frame_J(std::deque<frame_info::ptr> &frame_infos,
                          int index);

        void clac_camera_J(std::deque<frame_info::ptr> &frame_infos,
                           feature_manger &feature_infos);

        void clac_prior_J(std::deque<frame_info::ptr> &frame_infos,
                          feature_manger &feature_infos);

        void rotate_frame(std::deque<frame_info::ptr> &frame_infos,
                          feature_manger &feature_infos);
        void do_init_solve(std::deque<frame_info::ptr> &frame_infos,
                           feature_manger &feature_infos,
                           bool enable_camera_factor);

        void clac_ext_camera(std::deque<frame_info::ptr> &frame_infos, feature_manger &feature_infos);

    public:
        void solve(std::deque<frame_info::ptr> &frame_infos,
                   feature_manger &feature_infos);

        void init_solve(std::deque<frame_info::ptr> &frame_infos,
                        feature_manger &feature_infos);
        void marginalization(std::deque<frame_info::ptr> &frame_infos, feature_manger &feature_infos);

        solver();
    };

} // namespace lvio_2d
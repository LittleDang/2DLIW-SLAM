#pragma once
#include "timerAndColor/color.h"
#include <Eigen/Dense>
#include <ros/ros.h>
#include <yaml-cpp/yaml.h>

#define PARAM(name) (lvio_2d::param::manager::get_param_manager()->name)

namespace lvio_2d
{
    enum TRAJECTORY_STATUS
    {
        INITIALIZING,
        TRACKING
    };
    namespace param
    {
        class manager
        {
        public:
            using ptr = std::shared_ptr<manager>;

            std::string wheel_odom_topic;
            std::string laser_topic;
            std::string imu_topic;
            std::string camera_topic;

            Eigen::Isometry3d T_imu_to_camera;
            Eigen::Isometry3d T_imu_to_laser;
            Eigen::Isometry3d T_imu_to_wheel;

            Eigen::Vector3d imu_noise_acc_sigma;
            Eigen::Vector3d imu_bias_acc_sigma;
            Eigen::Vector3d imu_noise_gyro_sigma;
            Eigen::Vector3d imu_bias_gyro_sigma;
            Eigen::Vector3d wheel_sigma;

            Eigen::Vector2d camera_sigma;
            double max_camera_reproject_error;
            double max_camera_feature_dis;

            int max_feature_num;

            int slide_window_size;
            double p_motion_threshold;
            double q_motion_threshold;

            double g;
            Eigen::Matrix3d camera_K;

            double manifold_p_sigma;
            double manifold_q_sigma;

            double w_laser_each_scan;
            double h_laser_each_scan;
            double laser_resolution;

            double line_continuous_threshold;
            double line_max_tolerance_angle;
            double line_min_len;
            double line_max_dis;
            double line_to_line_sigma;

            bool enable_camera;
            bool enable_laser;

            bool enable_camera_vis;
            bool enable_laser_vis;

            double feature_min_dis;
            int FPS;
            double min_delta_t;

            double key_frame_p_motion_threshold;
            double key_frame_q_motion_threshold;

            double a_res;
            double d_res;
            int submap_count;
            int laser_loop_min_match_threshold;
            int loop_detect_min_interval;
            std::string output_dir;
            bool output_tum;
            bool use_ground_p_factor;
            bool use_ground_q_factor;
            double verify_loop_rate;
            double loop_max_dis;

            double loop_edge_k;
            double ref_motion_filter_p;
            double ref_motion_filter_q;
            double loop_max_tf_p;
            double loop_max_tf_q;

            bool fast_mode;
            Eigen::Vector3d loop_sigma_p;
            Eigen::Vector3d loop_sigma_q;



            int ref_n_accumulation;
            static manager::ptr get_param_manager();

            static void update_T_imu_to_camera(const Eigen::Isometry3d &T_i_c)
            {
                get_param_manager()->T_imu_to_camera = T_i_c;
            }

        private:
            manager(const ros::NodeHandle &nh);
            bool check_param();
        };

    } // namespace param
} // namespace lvio_2d
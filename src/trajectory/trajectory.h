#pragma once
#include "factor/camera_factor.h"
#include "factor/imu_preintegraption.h"
#include "factor/solver.h"
#include "factor/wheel_odom_preintegration.h"
#include "trajectory/camera_manager.h"
#include "trajectory/keyframe_manager.h"
#include "trajectory/laser_manager.h"
#include "trajectory/sensor.h"
#include "trajectory/trajectory_type.h"
#include "utilies/record.h"
#include "utilies/utilies.h"
#include <deque>
namespace lvio_2d
{
    class trajectory
    {
    public:
        using ptr = std::shared_ptr<trajectory>;

        trajectory();
        ~trajectory();

        // 数据进入的接口
        void add_sensor_data(sensor::wheel_odom::u_ptr &wheel_odom_data_ptr);
        void add_sensor_data(sensor::imu::u_ptr &imu_data_ptr);
        void add_sensor_data(sensor::laser::u_ptr &laser_data_ptr);
        void add_sensor_data(sensor::camera::u_ptr &camera_image_ptr);

    private:
        imu_preintegraption imu_preintegraption_;
        wheel_odom_preintegration wheel_odom_preintegration_;
        lvio_2d::camera_manager camera_manger_;
        lvio_2d::laser_manager laser_manger_;
        lvio_2d::feature_manger feature_manger_;
        std::deque<frame_info::ptr> frame_infos;
        Eigen::Isometry3d last_keyframe_tf;

        // current status
        double current_time;
        double last_time;
        int current_index;
        int last_laser_index;
        int last_camera_index;
        Eigen::Matrix<double, 3, 1> current_p;
        Eigen::Matrix<double, 3, 1> current_q;
        Eigen::Matrix<double, 3, 1> current_v;
        Eigen::Matrix<double, 6, 1> current_bs;

        // 在local下的当前的角速度
        Eigen::Matrix<double, 3, 1> current_angular_local;
        TRAJECTORY_STATUS status;

        bool wheel_odom_inited;
        bool imu_inited;

        // solver
        solver opt_solver;

        // keyframe
        keyframe_manager keyframe_manager_;
        std::vector<Eigen::Vector3d> acc_concers;
        void init_current_status();
        void update_current_status(const Eigen::Isometry3d &delta_tf, const double &time);

        bool check_and_processing_initialize();
        void do_tracking();
        void estimate_features(const bool &remove);
        void pop_frame(int k);
        void pop_frame_for_tracking();

        void filter_outlier_world_point();

        std::vector<Eigen::Isometry3d> odom_path;
        std::vector<Eigen::Isometry3d> tracking_path;

        void update_path_and_pub(const Eigen::Isometry3d &delta_tf);
        void show_lastest_image(const std::string &win_name);

        // log
        std::fstream o_fstream;
        std::string o_path;
        record recorder;
    };

}; // namespace lvio_2d
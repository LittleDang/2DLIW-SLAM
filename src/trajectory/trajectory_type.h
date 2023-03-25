#pragma once
#include "factor/imu_preintegraption.h"
#include "factor/wheel_odom_preintegration.h"
#include "trajectory/laser_manager.h"
#include "utilies/utilies.h"

namespace lvio_2d
{
    struct frame_info
    {
        enum frame_type
        {
            laser = 0,
            camera = 1,
            unknow = 2
        };
        double time;

        // at i frame
        using ptr = std::shared_ptr<frame_info>;
        using w_ptr = std::weak_ptr<frame_info>;

        Eigen::Matrix<double, 3, 1> p;
        Eigen::Matrix<double, 3, 1> q;
        Eigen::Matrix<double, 3, 1> v;
        Eigen::Matrix<double, 6, 1> bs;

        imu_preint_result::ptr imu_observation_reslut;          // i-1~i
        wheel_odom_preint_result::ptr wheel_observation_reslut; // i-1~i

        laser_match::ptr laser_match_ptr;
        cv::Mat image;

        frame_type type;
        bool is_key_frame;
        std::vector<Eigen::Vector3d> laser_concers; // only use for concers

        Eigen::Matrix<double, 6, 6> sqrt_H;

        static ptr create(const double &time_,
                          const Eigen::Matrix<double, 3, 1> &p_,
                          const Eigen::Matrix<double, 3, 1> &q_,
                          const Eigen::Matrix<double, 3, 1> &v_,
                          const Eigen::Matrix<double, 6, 1> &bs_,
                          const imu_preint_result::ptr &imu_observation_reslut_,
                          const wheel_odom_preint_result::ptr &wheel_observation_reslut_)
        {
            ptr ret(new frame_info());
            ret->p = p_;
            ret->q = q_;
            ret->v = v_;
            ret->bs = bs_;
            ret->imu_observation_reslut = imu_observation_reslut_;
            ret->wheel_observation_reslut = wheel_observation_reslut_;
            ret->time = time_;
            ret->type = unknow;
            ret->is_key_frame = false;
            ret->sqrt_H = Eigen::Matrix<double, 6, 6>::Identity();
            return ret;
        }

        void add_laser_match(const laser_match::ptr &laser_match_ptr_)
        {
            laser_match_ptr = laser_match_ptr_;
            type = laser;
        }
        void set_key_frame()
        {
            is_key_frame = true;
        }
        void set_acc_concers(const std::vector<Eigen::Vector3d> &laser_concers_)
        {
            laser_concers = laser_concers_;
        }
    };
} // namespace lvio_2d

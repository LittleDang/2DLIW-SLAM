#pragma once
#include "factor/factor_common.h"
#include "trajectory/sensor.h"
namespace lvio_2d
{
    struct wheel_noise
    {
        using const_ptr = std::shared_ptr<const wheel_noise>;

        Eigen::Matrix3d wheel_cov;

        static wheel_noise::const_ptr get_wheel_noise()
        {
            static wheel_noise::const_ptr ret = nullptr;
            if (!ret)
                ret = std::make_shared<const wheel_noise>();
            return ret;
        }
        wheel_noise()
        {
            wheel_cov = convert::disdiagonal2<double, 3>(PARAM(wheel_sigma));
        }
    };

    struct wheel_odom_preint_result
    {
        using ptr = std::shared_ptr<wheel_odom_preint_result>;
        Eigen::Isometry3d delta_Tij;
        Eigen::Matrix3d sqrt_inverse_P;
        double Dt;
        wheel_odom_preint_result(const Eigen::Isometry3d &delta_Tij,
                                 const Eigen::Matrix3d &sqrt_inverse_P,
                                 const double &Dt) : sqrt_inverse_P(sqrt_inverse_P), delta_Tij(delta_Tij), Dt(Dt)
        {
        }
        static ptr create(const Eigen::Isometry3d &delta_Tij,
                          const Eigen::Matrix3d &sqrt_inverse_P,
                          const double &Dt)
        {
            return std::make_shared<wheel_odom_preint_result>(delta_Tij, sqrt_inverse_P, Dt);
        }
    };

    class wheel_odom_preintegration
    {
    public:
        wheel_odom_preintegration()
        {
            v = omega = Eigen::Vector3d::Zero();
            reset_wheel_odom_measure(-1);
        }
        void reset_wheel_odom_measure(
            const double &time)
        {
            last_update_time = time;

            delta_Tij = Eigen::Isometry3d::Identity();
            cov = Eigen::Matrix<double, 6, 6>::Zero();

            Dt = 0;
        }
        bool add_wheel_odom_measure(const sensor::wheel_odom::u_ptr &data)
        {

            if (last_update_time < 0)
            {
                last_add_wheel_odom_pose = data->pose;

                last_add_wheel_odom_time = data->time_stamp;
                last_update_time = data->time_stamp;

                delta_Tij = Eigen::Isometry3d::Identity();
                v = omega = Eigen::Vector3d::Zero();
                return false;
            }

            double dt = data->time_stamp - last_add_wheel_odom_time;
            auto [delta_p, delta_theta] = lie::log_SE3(
                last_add_wheel_odom_pose.inverse() * data->pose);

            if (dt < 0.05)
                return false;

           
            v = delta_p / dt;
            omega = delta_theta / dt;
            double update_dt = data->time_stamp - last_update_time;
            if (update_dt < 0)
                ROS_WARN("call by add_wheel_odom_measure");
            update_by_v(update_dt);

            last_add_wheel_odom_pose = data->pose;

            last_add_wheel_odom_time = data->time_stamp;
            last_update_time = data->time_stamp;
            return true;
        }

        void update_only_t(const double &time)
        {
            if (last_update_time < 0)
            {
                return;
            }
            double update_dt = time - last_update_time;
            if (update_dt < 0)
                ROS_WARN("call by update_only_t");
            update_by_v(update_dt);
            last_update_time = time;
        }
        wheel_odom_preint_result::ptr get_preintegraption_result()
        {
            Eigen::Matrix3d cov = Eigen::Matrix3d::Zero();
            auto [delta_p, delta_q] = lie::log_SE3(delta_Tij);
            double len_norm = std::max(delta_p.squaredNorm(), 0.005 * 0.005);
            double delta_yaw_norm = std::max(delta_q.squaredNorm(), 0.005 * 0.005);

            Eigen::Matrix3d k = Eigen::Matrix3d::Identity();
            k(0, 0) = len_norm;
            k(1, 1) = len_norm;
            k(2, 2) = delta_yaw_norm;
            cov = wheel_noise::get_wheel_noise()->wheel_cov * k;
            Eigen::Matrix3d sqrt_info = Eigen::LLT<Eigen::Matrix3d>(cov.inverse()).matrixL().transpose();
            return wheel_odom_preint_result::create(delta_Tij, sqrt_info, Dt);
        }

    private:
        Eigen::Isometry3d delta_Tij;

        double last_update_time;
        double last_add_wheel_odom_time;

        Eigen::Isometry3d last_add_wheel_odom_pose;

        Eigen::Vector3d omega;
        Eigen::Vector3d v;

        Eigen::Matrix<double, 6, 6> cov;

        double Dt;
        void update_by_v(const double &dt)
        {
            if (dt <= 0 || dt >= 10)
            {
                ROS_WARN("dt <= 0) || dt>=10 :%lf", dt);
                return;
            }

            Dt = Dt + dt;
            auto delta_T = lie::make_tf<double>(v * dt, omega * dt);
            delta_Tij = delta_Tij * delta_T;
        }
    };
} // namespace lvio_2d
#pragma once
#include "factor/factor_common.h"
#include <ceres/rotation.h>
namespace lvio_2d
{
    struct laser_noise
    {
        double sqrt_info;
        double angle_info;
        double p_info;
        using const_ptr = std::shared_ptr<const laser_noise>;
        static const_ptr get_laser_noise()
        {
            static const_ptr ret = nullptr;
            if (!ret)
                ret = std::make_shared<const laser_noise>();
            return ret;
        }
        laser_noise()
        {
            sqrt_info = 1.0 / PARAM(line_to_line_sigma);
            angle_info = 1.0 / 0.001;
            p_info = 1.0 / 0.005;
        }
    };
    struct laser_factor
    {
        Eigen::Vector3d l1_p1, l1_p2;
        Eigen::Vector3d l2_p1, l2_p2;
        double len1, len2, sum;
        laser_factor(const Eigen::Vector3d &l1_p1_, const Eigen::Vector3d &l1_p2_,
                     const Eigen::Vector3d &l2_p1_, const Eigen::Vector3d &l2_p2_)
        {
            l1_p1 = l1_p1_;
            l1_p2 = l1_p2_;
            l2_p1 = l2_p1_;
            l2_p2 = l2_p2_;
            len1 = (l1_p1_ - l1_p2_).norm();
            len2 = (l2_p1_ - l2_p2_).norm();
            double tmp = std::min(len1, len2);
            sum = tmp / 2.0 / 0.02;
            sum = std::sqrt(sum);
        }

        template <typename T>
        bool operator()(const T *const p_w_i, const T *const theta_w_i,
                        const T *const p_w_j, const T *const theta_w_j,
                        T *res) const
        {
            using vector3 = Eigen::Matrix<T, 3, 1>;
            using matrix3 = Eigen::Matrix<T, 3, 3>;

            const Eigen::Map<const vector3> p_i(p_w_i);
            const Eigen::Map<const vector3> theta_i(theta_w_i);

            const Eigen::Map<const vector3> p_j(p_w_j);
            const Eigen::Map<const vector3> theta_j(theta_w_j);

            Eigen::Transform<T, 3, Eigen::Isometry> T_i_l = PARAM(T_imu_to_laser).template cast<T>();
            Eigen::Transform<T, 3, Eigen::Isometry> T_l_i = T_i_l.inverse();

            Eigen::Transform<T, 3, Eigen::Isometry> T_w_i =
                lie::make_tf<T>(p_i, theta_i) * T_i_l;
            Eigen::Transform<T, 3, Eigen::Isometry> T_w_j =
                lie::make_tf<T>(p_j, theta_j) * T_i_l;

            Eigen::Matrix<T, 3, 1> l2_point1 = T_w_j * l2_p1.cast<T>();
            Eigen::Matrix<T, 3, 1> l2_point2 = T_w_j * l2_p2.cast<T>();

            Eigen::Matrix<T, 3, 1> l1_point1 = T_w_i * l1_p1.cast<T>();
            Eigen::Matrix<T, 3, 1> l1_point2 = T_w_i * l1_p2.cast<T>();

            l2_point1(2) = T(0);
            l2_point2(2) = T(0);

            l1_point1(2) = T(0);
            l1_point2(2) = T(0);

            T dis1 = e_laser::dis_from_line<T>(l2_point1, l1_point1, l1_point2);
            T dis2 = e_laser::dis_from_line<T>(l2_point2, l1_point1, l1_point2);

            T e1 = T(laser_noise::get_laser_noise()->sqrt_info) * (dis1);
            T e2 = T(laser_noise::get_laser_noise()->sqrt_info) * (dis2);
            res[0] =
                T(sum) * (e1);
            res[1] = T(sum) * (e2);

            return true;
        }

        static ceres::CostFunction *
        Create(const Eigen::Vector3d &l1_p1_, const Eigen::Vector3d &l1_p2_,
               const Eigen::Vector3d &l2_p1_, const Eigen::Vector3d &l2_p2_)

        {
            return new ceres::AutoDiffCostFunction<laser_factor, 2, 3, 3, 3, 3>(
                new laser_factor(l1_p1_, l1_p2_, l2_p1_, l2_p2_));
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} // namespace lvio_2d
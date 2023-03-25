#pragma once
#include "factor/factor_common.h"
#include "factor/wheel_odom_preintegration.h"
namespace lvio_2d
{
    struct wheel_odom_factor
    {
        wheel_odom_preint_result::ptr wheel_odom_preint_result_;
        wheel_odom_factor(const wheel_odom_preint_result::ptr &wheel_odom_preint_result_) : wheel_odom_preint_result_(wheel_odom_preint_result_)
        {
        }
        template <typename T>
        bool operator()(const T *const p_w_i,
                        const T *const theta_w_i,
                        const T *const p_w_j,
                        const T *const theta_w_j,
                        T *res) const
        {
            using vector3 = Eigen::Matrix<T, 3, 1>;
            using vector6 = Eigen::Matrix<T, 6, 1>;
            using matrix3 = Eigen::Matrix<T, 3, 3>;

            const Eigen::Map<const vector3> pi(p_w_i);
            const Eigen::Map<const vector3> thetai(theta_w_i);

            const Eigen::Map<const vector3> pj(p_w_j);
            const Eigen::Map<const vector3> thetaj(theta_w_j);

            Eigen::Transform<T, 3, Eigen::Isometry> T_i_w = PARAM(T_imu_to_wheel).template cast<T>();

            Eigen::Transform<T, 3, Eigen::Isometry> tf_i = lie::make_tf<T>(pi, thetai) * T_i_w;
            Eigen::Transform<T, 3, Eigen::Isometry> tf_j = lie::make_tf<T>(pj, thetaj) * T_i_w;

            Eigen::Transform<T, 3, Eigen::Isometry> w_tf_ij = tf_i.inverse() * tf_j;

            auto [p, q] = lie::log_SE3(w_tf_ij);
            auto [op, oq] = lie::log_SE3<T>(wheel_odom_preint_result_->delta_Tij.cast<T>());

            T o_len = ceres::sqrt(op(0) * op(0) + op(1) * op(1));
            T len = ceres::sqrt(p(0) * p(0) + p(1) * p(1));

            vector3 o_dir(op(0), op(1), T(0));
            vector3 dir(p(0), p(1), T(0));
            T angle = T(0);
            if (o_dir.norm() > T(0.0001) && dir.norm() > T(0.0001))
            {
                o_dir.normalize();
                dir.normalize();

                T sinn = o_dir.cross(dir).norm();
                angle = ceres::asin(sinn);
            }
            else
            {
                angle = dir.norm();
            }

            if (len < T(0.0001) || o_len < T(0.0001))
                res[0] = T(wheel_odom_preint_result_->sqrt_inverse_P(0, 0)) * len;
            else
                res[0] = T(wheel_odom_preint_result_->sqrt_inverse_P(0, 0)) * (o_len - len);
            res[1] = T(wheel_odom_preint_result_->sqrt_inverse_P(1, 1)) * (angle);
            if (q.norm() < T(0.001) || oq.norm() < T(0.001))
            {
                res[2] = T(wheel_odom_preint_result_->sqrt_inverse_P(2, 2)) * q.norm();
            }
            else
            {
                res[2] = T(wheel_odom_preint_result_->sqrt_inverse_P(2, 2)) * (oq.norm() - q.norm());
            }

            return true;
        }

        static ceres::CostFunction *Create(
            const wheel_odom_preint_result::ptr &wheel_odom_preint_result_)
        {
            return new ceres::AutoDiffCostFunction<wheel_odom_factor, 3, 3, 3, 3, 3>(
                new wheel_odom_factor(wheel_odom_preint_result_));
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} // namespace lvio_2d
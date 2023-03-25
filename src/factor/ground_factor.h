#pragma once
#include "factor/factor_common.h"
namespace lvio_2d
{
    struct ground_noise
    {
        double manifold_p_sqrt_info;
        double manifold_q_sqrt_info;

        using const_ptr = std::shared_ptr<const ground_noise>;
        static const_ptr get_ground_noise()
        {
            static const_ptr ret = nullptr;
            if (!ret)
                ret = std::make_shared<const ground_noise>();
            return ret;
        }
        ground_noise()
        {
            manifold_p_sqrt_info = 1.0 / PARAM(manifold_p_sigma);
            manifold_q_sqrt_info = 1.0 / PARAM(manifold_q_sigma);
        }
    };

    struct ground_factor_p
    {
        template <typename T>
        bool operator()(const T *const p_w_i,
                        const T *const theta_w_i,
                        T *res) const
        {
            using vector3 = Eigen::Matrix<T, 3, 1>;
            using vector6 = Eigen::Matrix<T, 6, 1>;
            using matrix3 = Eigen::Matrix<T, 3, 3>;

            Eigen::Transform<T, 3, Eigen::Isometry> tf_w_i = lie::make_tf<T>(
                Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p_w_i),
                Eigen::Map<const Eigen::Matrix<T, 3, 1>>(theta_w_i));

            Eigen::Transform<T, 3, Eigen::Isometry> T_i_w = PARAM(T_imu_to_wheel).template cast<T>();

            Eigen::Transform<T, 3, Eigen::Isometry> tf_w_o = tf_w_i * T_i_w;

            T dis_from_plane = tf_w_o.matrix()(2, 3);

            res[0] = T(ground_noise::get_ground_noise()->manifold_p_sqrt_info) * dis_from_plane;
            return true;
        }
        static ceres::CostFunction *Create()
        {
            return new ceres::AutoDiffCostFunction<ground_factor_p, 1, 3, 3>(
                new ground_factor_p());
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    struct ground_factor_q
    {
        template <typename T>
        bool operator()(const T *const p_w_i,
                        const T *const theta_w_i,
                        T *res) const
        {
            using vector3 = Eigen::Matrix<T, 3, 1>;
            using vector6 = Eigen::Matrix<T, 6, 1>;
            using matrix3 = Eigen::Matrix<T, 3, 3>;

            Eigen::Transform<T, 3, Eigen::Isometry> tf_w_i = lie::make_tf<T>(
                Eigen::Map<const Eigen::Matrix<T, 3, 1>>(p_w_i),
                Eigen::Map<const Eigen::Matrix<T, 3, 1>>(theta_w_i));

            Eigen::Transform<T, 3, Eigen::Isometry> T_i_w = PARAM(T_imu_to_wheel).template cast<T>();

            Eigen::Transform<T, 3, Eigen::Isometry> tf_w_o = tf_w_i * T_i_w;

            vector3 ABC(T(0), T(0), T(1));
            vector3 z_axis = tf_w_o.matrix().template block<3, 1>(0, 2);
            T sinn = z_axis.cross(ABC).norm();
            T angle = ceres::asin(sinn);
            res[0] = T(ground_noise::get_ground_noise()->manifold_q_sqrt_info) * angle;
            return true;
        }
        static ceres::CostFunction *Create()
        {
            return new ceres::AutoDiffCostFunction<ground_factor_q, 1, 3, 3>(
                new ground_factor_q());
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} // namespace lvio_2d
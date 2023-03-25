#pragma once
#include "factor/factor_common.h"
namespace lvio_2d
{
    struct camera_noise
    {
        Eigen::Matrix2d cov;
        Eigen::Vector2d camera_sigma;
        Eigen::Matrix2d sqrt_info;
        using const_ptr = std::shared_ptr<const camera_noise>;
        static const_ptr get_camera_noise()
        {
            static const_ptr ret = nullptr;
            if (!ret)
                ret = std::make_shared<const camera_noise>();
            return ret;
        }
        camera_noise()
        {
            camera_sigma = PARAM(camera_sigma);
            camera_sigma /= PARAM(camera_K)(0, 0);
            cov = convert::disdiagonal2<double, 2>(camera_sigma);
            sqrt_info = Eigen::LLT<Eigen::Matrix2d>(
                            cov.inverse())
                            .matrixL()
                            .transpose();
        }
    };
    struct camera_factor
    {
        Eigen::Vector3d cam_point;
        camera_factor(const Eigen::Vector3d &cam_p) : cam_point(cam_p)
        {
        }

        template <typename T>
        bool operator()(const T *const p_w_i,
                        const T *const theta_w_i,
                        const T *const world_point,
                        T *res) const
        {
            using vector3 = Eigen::Matrix<T, 3, 1>;
            using matrix3 = Eigen::Matrix<T, 3, 3>;

            const Eigen::Map<const vector3> p_i(p_w_i);
            const Eigen::Map<const vector3> theta_i(theta_w_i);
            const Eigen::Map<const vector3> wp(world_point);

            Eigen::Transform<T, 3, Eigen::Isometry> T_i_c =
                PARAM(T_imu_to_camera).cast<T>();

            Eigen::Transform<T, 3, Eigen::Isometry>
                T_w_i = lie::make_tf<T>(p_i, theta_i);

            Eigen::Transform<T, 3, Eigen::Isometry> T_c_w = (T_w_i * T_i_c).inverse();

            Eigen::Map<Eigen::Matrix<T, 2, 1>> res_vec(res);
            res_vec = Eigen::Matrix<T, 2, 1>::Zero();

            vector3 reprojcet_j = T_c_w * wp;
            reprojcet_j /= reprojcet_j(2);

            res_vec = cam_point.template cast<T>().template block<2, 1>(0, 0) - reprojcet_j.template block<2, 1>(0, 0);

            res_vec = camera_noise::get_camera_noise()->sqrt_info.template cast<T>() *
                      res_vec;
            return true;
        }

        static ceres::CostFunction *Create(const Eigen::Vector3d &cam_p)
        {
            return new ceres::AutoDiffCostFunction<camera_factor, 2, 3, 3, 3>(
                new camera_factor(cam_p));
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} // namespace lvio_2d
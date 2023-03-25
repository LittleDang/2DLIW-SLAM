#include "factor/camera_factor.h"
#include <ceres/rotation.h>
namespace lvio_2d
{
    struct point_factor
    {
        Eigen::Vector3d p1;
        Eigen::Vector3d p2;
        point_factor(const Eigen::Vector3d &p1_,
                     const Eigen::Vector3d &p2_)
        {
            p1 = p1_;
            p2 = p2_;
        }

        template <typename T>
        bool operator()(const T *const p_w_i, const T *const theta_w_i,
                        T *res) const
        {
            using vector3 = Eigen::Matrix<T, 3, 1>;
            using matrix3 = Eigen::Matrix<T, 3, 3>;

            const Eigen::Map<const vector3> p_i(p_w_i);
            const Eigen::Map<const vector3> theta_i(theta_w_i);

            Eigen::Transform<T, 3, Eigen::Isometry> T_i_j =
                lie::make_tf<T>(p_i, theta_i);

            Eigen::Matrix<T, 3, 1> p_error = p1.cast<T>() - T_i_j * p2.cast<T>();

            res[0] = T(100) * p_error(0);
            res[1] = T(100) * p_error(1);
            res[2] = T(100) * p_error(2);
            return true;
        }

        static ceres::CostFunction *Create(
            const Eigen::Vector3d &p1_,
            const Eigen::Vector3d &p2_)
        {
            return new ceres::AutoDiffCostFunction<point_factor, 3, 3, 3>(
                new point_factor(p1_, p2_));
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} // namespace lvio_2d
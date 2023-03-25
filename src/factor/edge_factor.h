#include "factor/camera_factor.h"
namespace lvio_2d
{
    struct edge_noise
    {
        Eigen::Matrix<double, 6, 6> J;
        using const_ptr = std::shared_ptr<const edge_noise>;
        static const_ptr get_edge_noise()
        {
            static const_ptr ret = nullptr;
            if (!ret)
                ret = std::make_shared<const edge_noise>();
            return ret;
        }
        edge_noise()
        {
            J = Eigen::Matrix<double, 6, 6>::Identity();
            J(0, 0) = 1.0 / PARAM(loop_sigma_p(0));
            J(1, 2) = 1.0 / PARAM(loop_sigma_p(1));
            J(2, 2) = 1.0 / PARAM(loop_sigma_p(2));

            J(3, 3) = 1.0 / PARAM(loop_sigma_q(0));
            J(4, 4) = 1.0 / PARAM(loop_sigma_q(1));
            J(5, 5) = 1.0 / PARAM(loop_sigma_q(2));
        }
    };
    struct prior_factor
    {
        Eigen::Vector3d p, q;
        Eigen::Matrix<double, 6, 6> J;
        prior_factor(const Eigen::Vector3d &p_,
                     const Eigen::Vector3d &q_,
                     const Eigen::Matrix<double, 6, 6> &J_)
        {
            J = J_;
            p = p_;
            q = q_;
        }

        template <typename T>
        bool operator()(const T *const p_w_i,
                        const T *const theta_w_i,
                        T *res) const
        {
            using vector3 = Eigen::Matrix<T, 3, 1>;
            using vector6 = Eigen::Matrix<T, 6, 1>;
            using matrix3 = Eigen::Matrix<T, 3, 3>;

            const Eigen::Map<const vector3> pi(p_w_i);
            const Eigen::Map<const vector3> qi(theta_w_i);

            Eigen::Map<vector3> res_p(res);
            Eigen::Map<vector3> res_theta(res + 3);
            Eigen::Map<vector6> res_all(res);

            res_p[0] = T(p(0)) - pi(0);
            res_p[1] = T(p(1)) - pi(1);
            res_p[2] = T(p(2)) - pi(2);

            res_theta[0] = T(q(0)) - qi(0);
            res_theta[1] = T(q(1)) - qi(1);
            res_theta[2] = T(q(2)) - qi(2);

            res_all = J.cast<T>() * res_all;
            return true;
        }

        static ceres::CostFunction *Create(
            const Eigen::Vector3d &p_,
            const Eigen::Vector3d &q_,
            const Eigen::Matrix<double, 6, 6> &J_)
        {
            return new ceres::AutoDiffCostFunction<prior_factor, 6, 3, 3>(
                new prior_factor(p_, q_, J_));
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    struct edge_factor
    {
        Eigen::Isometry3d tf12;
        double weight;
        edge_factor(const Eigen::Isometry3d &tf12_, double weight_ = 1.0) : tf12(tf12_)
        {
            weight = weight_;
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

            Eigen::Map<vector3> res_p(res);
            Eigen::Map<vector3> res_theta(res + 3);
            Eigen::Map<vector6> res_all(res);

            Eigen::Transform<T, 3, Eigen::Isometry> tf_i = lie::make_tf<T>(pi, thetai);
            Eigen::Transform<T, 3, Eigen::Isometry> tf_j = lie::make_tf<T>(pj, thetaj);

            Eigen::Transform<T, 3, Eigen::Isometry> error = tf_j.inverse() * tf_i * tf12.template cast<T>();

            std::tie(res_p, res_theta) = lie::log_SE3<T>(error);
            res_all = T(weight) * edge_noise::get_edge_noise()->J.cast<T>() * res_all;
            return true;
        }

        static ceres::CostFunction *Create(
            const Eigen::Isometry3d &tf12_, double weight_ = 1.0)
        {
            return new ceres::AutoDiffCostFunction<edge_factor, 6, 3, 3, 3, 3>(
                new edge_factor(tf12_, weight_));
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

    struct loop_edge_factor
    {
        Eigen::Isometry3d w_tf12;
        double weight;
        loop_edge_factor(const Eigen::Isometry3d &tf12_, double weight_ = 1.0)
        {
            weight = weight_;
            w_tf12 = PARAM(T_imu_to_wheel).inverse() * tf12_ * PARAM(T_imu_to_wheel);
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

            Eigen::Transform<T, 3, Eigen::Isometry> tf_i = lie::make_tf<T>(pi, thetai);
            Eigen::Transform<T, 3, Eigen::Isometry> tf_j = lie::make_tf<T>(pj, thetaj);

            Eigen::Transform<T, 3, Eigen::Isometry> tf_ij = tf_i.inverse() * tf_j;
            Eigen::Transform<T, 3, Eigen::Isometry> w_tf_ij = PARAM(T_imu_to_wheel).inverse().cast<T>() * tf_ij * PARAM(T_imu_to_wheel).cast<T>();

            auto [p, q] = lie::log_SE3(w_tf_ij);
            auto [op, oq] = lie::log_SE3<T>(w_tf12.cast<T>());

            res[0] = weight * T(edge_noise::get_edge_noise()->J(0, 0)) * (p[0] - op[0]);
            res[1] = weight * T(edge_noise::get_edge_noise()->J(1, 1)) * (p[1] - op[1]);
            res[2] = weight * T(edge_noise::get_edge_noise()->J(5, 5)) * (q[2] - oq[2]);

            res[3] = res[4] = res[5] = T(0);
            return true;
        }

        static ceres::CostFunction *Create(
            const Eigen::Isometry3d &tf12_, double weight_ = 1.0)
        {
            return new ceres::AutoDiffCostFunction<loop_edge_factor, 6, 3, 3, 3, 3>(
                new loop_edge_factor(tf12_, weight_));
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} // namespace lvio_2d
#pragma once
#include "utilies/common.h"
#include "utilies/params.h"
#include <ceres/ceres.h>
namespace lvio_2d
{
    namespace magic_number_X
    {
        constexpr int alpha_len = 3;
        constexpr int beta_len = 3;
        constexpr int gamma_len = 3;
        constexpr int ba_len = 3;
        constexpr int bw_len = 3;

        constexpr int all_status_len = alpha_len + beta_len + gamma_len + ba_len + bw_len;
        constexpr int alpha_index = 0;
        constexpr int beta_index = alpha_len;
        constexpr int gamma_index = alpha_len + beta_len;
        constexpr int ba_index = alpha_len + beta_len + gamma_len;
        constexpr int bw_index = alpha_len + beta_len + gamma_len + ba_len;

        constexpr int na_len = 3;
        constexpr int nw_len = 3;
        constexpr int nba_len = 3;
        constexpr int nbw_len = 3;

        constexpr int all_noise_len = na_len + nw_len + nba_len + nbw_len;
        constexpr int na_index = 0;
        constexpr int nw_index = na_len;
        constexpr int nba_index = na_len + nw_len;
        constexpr int nbw_index = na_len + nw_len + nba_len;

    } // namespace magic_number_X

    namespace factor
    {
        class so3_parameterization
        {
        public:
            template <typename T>
            bool operator()(const T *theta_radians, const T *delta_theta_radians,
                            T *theta_radians_plus_delta) const
            {
                Eigen::Map<const Eigen::Matrix<T, 3, 1>> x(theta_radians);
                Eigen::Map<const Eigen::Matrix<T, 3, 1>> delte_x(delta_theta_radians);
                Eigen::Map<Eigen::Matrix<T, 3, 1>> x_plus_delte(theta_radians_plus_delta);

                Eigen::Matrix<T, 3, 1> tmp = x + delte_x;
                lie::normalize_so3<T>(tmp);

                x_plus_delte = tmp;
                return true;
            }

            static ceres::LocalParameterization *Create()
            {
                return (new ceres::AutoDiffLocalParameterization<so3_parameterization,
                                                                 3, 3>);
            }
        };

        class g_parameterization
        {
        public:
            template <typename T>
            bool operator()(const T *theta_radians, const T *delta_theta_radians,
                            T *theta_radians_plus_delta) const
            {
                Eigen::Map<const Eigen::Matrix<T, 3, 1>> x(theta_radians);
                Eigen::Map<const Eigen::Matrix<T, 3, 1>> delta_x(delta_theta_radians);
                Eigen::Map<Eigen::Matrix<T, 3, 1>> x_plus_delta(theta_radians_plus_delta);

                x_plus_delta = lie::exp_so3<T>(delta_x) * x;
                return true;
            }

            static ceres::LocalParameterization *Create()
            {
                return (new ceres::AutoDiffLocalParameterization<g_parameterization,
                                                                 3, 3>);
            }
        };

    } // namespace factor

} // namespace lvio_2d

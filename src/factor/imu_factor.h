#pragma once
#include "factor/factor_common.h"
#include "factor/imu_preintegraption.h"
namespace lvio_2d
{
    using namespace magic_number_X;
    struct imu_factor
    {
        imu_preint_result::ptr imu_preint_result_;
        imu_factor(const imu_preint_result::ptr &imu_preInt_result_) : imu_preint_result_(imu_preInt_result_)
        {
        }
        template <typename T>
        bool operator()(const T *const p_w_i,
                        const T *const theta_w_i,
                        const T *const v_w_i,
                        const T *const bs_w_i,
                        const T *const p_w_j,
                        const T *const theta_w_j,
                        const T *const v_w_j,
                        const T *const bs_w_j,
                        T *res) const
        {
            using vector3 = Eigen::Matrix<T, 3, 1>;
            using vectorN = Eigen::Matrix<T, all_status_len, 1>;
            using matrix3 = Eigen::Matrix<T, 3, 3>;
            using matrixN = Eigen::Matrix<T, all_status_len, all_status_len>;

            const Eigen::Map<const vector3> pi(p_w_i);
            const Eigen::Map<const vector3> vi(v_w_i);
            const Eigen::Map<const vector3> thetai(theta_w_i);
            const Eigen::Map<const vector3> bai(bs_w_i);
            const Eigen::Map<const vector3> bwi(bs_w_i + 3);

            const Eigen::Map<const vector3> pj(p_w_j);
            const Eigen::Map<const vector3> vj(v_w_j);
            const Eigen::Map<const vector3> thetaj(theta_w_j);
            const Eigen::Map<const vector3> baj(bs_w_j);
            const Eigen::Map<const vector3> bwj(bs_w_j + 3);

            T g_norm = T(PARAM(g));
            vector3 g = vector3(T(0), T(0), T(1));

            Eigen::Map<vector3> res_alpha(res + alpha_index);
            Eigen::Map<vector3> res_beta(res + beta_index);
            Eigen::Map<vector3> res_gamma(res + gamma_index);
            Eigen::Map<vector3> res_ba(res + ba_index);
            Eigen::Map<vector3> res_bw(res + bw_index);
            Eigen::Map<vectorN> res_all(res);

            vector3 alpha = imu_preint_result_->X.template block<alpha_len, 1>(alpha_index, 0).cast<T>();
            vector3 beta = imu_preint_result_->X.template block<beta_len, 1>(beta_index, 0).cast<T>();
            vector3 gamma = imu_preint_result_->X.template block<gamma_len, 1>(gamma_index, 0).cast<T>();
            vector3 ba = imu_preint_result_->X.template block<ba_len, 1>(ba_index, 0).cast<T>();
            vector3 bw = imu_preint_result_->X.template block<bw_len, 1>(bw_index, 0).cast<T>();
            T Dt = T(imu_preint_result_->Dt);

            matrix3 bk_R_w = lie::exp_so3<T>(-thetai);

            Eigen::Matrix<T, alpha_len, ba_len> alpha_J_ba =
                imu_preint_result_->J.block<alpha_len, ba_len>(alpha_index, ba_index).cast<T>();

            Eigen::Matrix<T, alpha_len, bw_len> alpha_J_bw =
                imu_preint_result_->J.block<alpha_len, bw_len>(alpha_index, bw_index).cast<T>();

            Eigen::Matrix<T, beta_len, ba_len> beta_J_ba =
                imu_preint_result_->J.block<beta_len, ba_len>(beta_index, ba_index).cast<T>();

            Eigen::Matrix<T, beta_len, bw_len> beta_J_bw =
                imu_preint_result_->J.block<beta_len, bw_len>(beta_index, bw_index).cast<T>();

            Eigen::Matrix<T, gamma_len, bw_len> gamma_J_bw =
                imu_preint_result_->J.block<gamma_len, bw_len>(gamma_index, bw_index).cast<T>();

            alpha = alpha + alpha_J_ba * (bai - ba) + alpha_J_bw * (bwi - bw);
            beta = beta + beta_J_ba * (bai - ba) + beta_J_bw * (bwi - bw);
            gamma = gamma + gamma_J_bw * (bwi - bw);

            res_alpha = alpha - bk_R_w * (pj - pi + 0.5 * g * g_norm * Dt * Dt - vi * Dt);
            res_beta = beta - bk_R_w * (vj + g * g_norm * Dt - vi);
            res_gamma = lie::log_SO3<T>(lie::exp_so3<T>(-gamma) * (lie::exp_so3<T>(-thetai) * lie::exp_so3<T>(thetaj)));
            res_ba = baj - bai;
            res_bw = bwj - bwi;

            matrixN sqrt_info = imu_preint_result_->sqrt_inverse_P.cast<T>();
            res_all = sqrt_info * res_all;

            return true;
        }

        static ceres::CostFunction *Create(
            const imu_preint_result::ptr &imu_preInt_result)
        {
            return new ceres::AutoDiffCostFunction<imu_factor, all_status_len,
                                                   alpha_len, gamma_len, beta_len, ba_len + bw_len,
                                                   alpha_len, gamma_len, beta_len, ba_len + bw_len>(
                new imu_factor(imu_preInt_result));
        }
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} // namespace lvio_2d
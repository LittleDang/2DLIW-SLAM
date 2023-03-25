#pragma once
#include "factor/factor_common.h"
#include "trajectory/sensor.h"
#include "utilies/common.h"
namespace lvio_2d
{
    using namespace magic_number_X;
    struct imu_noise
    {
        using const_ptr = std::shared_ptr<const imu_noise>;
        Eigen::Vector3d imu_noise_acc_sigma;
        Eigen::Vector3d imu_bias_acc_sigma;
        Eigen::Vector3d imu_noise_gyro_sigma;
        Eigen::Vector3d imu_bias_gyro_sigma;

        Eigen::Matrix<double, all_noise_len, all_noise_len> Q;
        static imu_noise::const_ptr get_imu_noise()
        {
            static imu_noise::const_ptr ret = nullptr;
            if (!ret)
                ret = std::make_shared<const imu_noise>();
            return ret;
        }
        imu_noise()
        {
            imu_noise_acc_sigma = PARAM(imu_noise_acc_sigma);
            imu_bias_acc_sigma = PARAM(imu_bias_acc_sigma);
            imu_noise_gyro_sigma = PARAM(imu_noise_gyro_sigma);
            imu_bias_gyro_sigma = PARAM(imu_bias_gyro_sigma);
            Q = Eigen::Matrix<double, all_noise_len, all_noise_len>::Zero();

            Q.block<na_len, na_len>(na_index, na_index) =
                convert::disdiagonal2<double, na_len>(imu_noise_acc_sigma);

            Q.block<nw_len, nw_len>(nw_index, nw_index) =
                convert::disdiagonal2<double, nw_len>(imu_noise_gyro_sigma);

            Q.block<nba_len, nba_len>(nba_index, nba_index) =
                convert::disdiagonal2<double, nba_len>(imu_bias_acc_sigma);

            Q.block<nbw_len, nbw_len>(nbw_index, nbw_index) =
                convert::disdiagonal2<double, nbw_len>(imu_bias_gyro_sigma);
        }
    };
    struct imu_preint_result
    {
        using ptr = std::shared_ptr<imu_preint_result>;
        Eigen::Matrix<double, all_status_len, 1> X;
        Eigen::Matrix<double, all_status_len, all_status_len> J;
        Eigen::Matrix<double, all_status_len, all_status_len> sqrt_inverse_P;
        Eigen::Vector3d linearized_ba, linearized_bw; // init value
        double Dt;
        imu_preint_result(const Eigen::Matrix<double, all_status_len, 1> &X,
                          const Eigen::Matrix<double, all_status_len, all_status_len> &J,
                          const Eigen::Matrix<double, all_status_len, all_status_len> &sqrt_inverse_P,
                          const double &Dt) : X(X), J(J), sqrt_inverse_P(sqrt_inverse_P), Dt(Dt)
        {
            linearized_ba = X.block<ba_len, 1>(ba_index, 0);
            linearized_bw = X.block<bw_len, 1>(bw_index, 0);
        }
        static ptr create(const Eigen::Matrix<double, all_status_len, 1> &X,
                          const Eigen::Matrix<double, all_status_len, all_status_len> &J,
                          const Eigen::Matrix<double, all_status_len, all_status_len> &sqrt_inverse_P,
                          const double &Dt)
        {
            return std::make_shared<imu_preint_result>(X, J, sqrt_inverse_P, Dt);
        }
        void update_value_with_bias(const Eigen::Vector3d &ba_new, const Eigen::Vector3d &bw_new)
        {
            X.block<ba_len, 1>(ba_index, 0) = ba_new;
            X.block<bw_len, 1>(bw_index, 0) = bw_new;

            Eigen::Vector3d delta_ba = ba_new - linearized_ba;
            Eigen::Vector3d delta_bw = bw_new - linearized_bw;

            Eigen::Vector3d alpha = X.template block<alpha_len, 1>(alpha_index, 0);
            Eigen::Vector3d beta = X.template block<beta_len, 1>(beta_index, 0);
            Eigen::Vector3d gamma = X.template block<gamma_len, 1>(gamma_index, 0);
            using T = double;

            Eigen::Matrix<T, alpha_len, ba_len> alpha_J_ba =
                J.block<alpha_len, ba_len>(alpha_index, ba_index);

            Eigen::Matrix<T, alpha_len, bw_len> alpha_J_bw =
                J.block<alpha_len, bw_len>(alpha_index, bw_index);

            Eigen::Matrix<T, beta_len, ba_len> beta_J_ba =
                J.block<beta_len, ba_len>(beta_index, ba_index);

            Eigen::Matrix<T, beta_len, bw_len> beta_J_bw =
                J.block<beta_len, bw_len>(beta_index, bw_index);

            Eigen::Matrix<T, gamma_len, bw_len> gamma_J_bw =
                J.block<gamma_len, bw_len>(gamma_index, bw_index);

            X.template block<alpha_len, 1>(alpha_index, 0) = alpha + alpha_J_ba * (delta_ba) + alpha_J_bw * (delta_bw);
            X.template block<beta_len, 1>(beta_index, 0) = beta + beta_J_ba * (delta_ba) + beta_J_bw * (delta_bw);
            X.template block<gamma_len, 1>(gamma_index, 0) = gamma + gamma_J_bw * (delta_bw);

            linearized_ba = ba_new;
            linearized_bw = bw_new;
        }
    };

    class imu_preintegraption
    {
    public:
        double Dt;
        imu_preintegraption()
        {
            reset_imu_measure(-1, {0, 0, 0}, {0, 0, 0});
        }
        void reset_imu_measure(
            const double &time, const Eigen::Vector3d &acc_bias, const Eigen::Vector3d &gyr_bias)
        {
            J = Eigen::Matrix<double, all_status_len, all_status_len>::Identity();
            P = Eigen::Matrix<double, all_status_len, all_status_len>::Identity() * 0.00001;
            X = Eigen::Matrix<double, all_status_len, 1>::Zero();
            X.template block<ba_len, 1>(ba_index, 0) = acc_bias;
            X.template block<bw_len, 1>(bw_index, 0) = gyr_bias;
            last_add_imu_time = time;
            index = 0;
            Dt = 0;
        }
        bool add_imu_measure(const sensor::imu::u_ptr &data)
        {
            if (last_add_imu_time == -1)
            {
                last_info = *data;
                last_add_imu_time = data->time_stamp;
                return false;
            }
            double dt = data->time_stamp - last_add_imu_time;
            update(dt);
            last_info = *data;
            last_add_imu_time = data->time_stamp;
            return true;
        }
        void update_only_t(const double &time)
        {
            if (last_add_imu_time == -1)
                return;
            double dt = time - last_add_imu_time;
            update(dt);
            last_add_imu_time = time;
        }
        imu_preint_result::ptr get_preintegraption_result()
        {
            Eigen::Matrix<double, all_status_len, all_status_len> sqrt_inverse_P = Eigen::LLT<Eigen::Matrix<double, all_status_len, all_status_len>>(P.inverse()).matrixL().transpose();

            return imu_preint_result::create(X, J, sqrt_inverse_P, Dt);
        }

    private:
        double imuAccNoise;
        double imuGyrNoise;

        double imuAccBiasN;
        double imuGyrBiasN;

        double last_add_imu_time;

        Eigen::Matrix<double, all_status_len, all_status_len> J; // jacobian matrix
        Eigen::Matrix<double, all_status_len, all_status_len> P; // covaiance matrix
        Eigen::Matrix<double, all_status_len, 1> X;              // status to be estimated

        sensor::imu last_info;

        int index = 0;
        void update(const double &dt)
        {
            Eigen::Vector3d last_alpha = X.block<alpha_len, 1>(alpha_index, 0);
            Eigen::Vector3d last_beta = X.block<beta_len, 1>(beta_index, 0);
            Eigen::Vector3d last_gamma = X.block<gamma_len, 1>(gamma_index, 0);
            Eigen::Vector3d last_ba = X.block<ba_len, 1>(ba_index, 0);
            Eigen::Vector3d last_bw = X.block<bw_len, 1>(bw_index, 0);
            Eigen::Matrix3d last_Rz = lie::exp_so3<double>(X.block<gamma_len, 1>(gamma_index, 0));

            Eigen::Vector3d hat_acc = last_info.acc;
            Eigen::Vector3d hat_gyro = last_info.gyro;

            // update X
            X.block<alpha_len, 1>(alpha_index, 0) = last_alpha + last_beta * dt + 0.5 * last_Rz * (hat_acc - last_ba) * dt * dt;
            X.block<beta_len, 1>(beta_index, 0) = last_beta + last_Rz * (hat_acc - last_ba) * dt;
            X.block<gamma_len, 1>(gamma_index, 0) = lie::log_SO3<double>(lie::exp_so3(last_gamma) * lie::exp_so3<double>((hat_gyro - last_bw) * dt));
            // update J and P and
            // 1.compute F
            Eigen::Matrix<double, all_status_len, all_status_len> F = Eigen::Matrix<double, all_status_len, all_status_len>::Zero();
            F.block<beta_len, beta_len>(alpha_index, beta_index) = Eigen::Matrix<double, beta_len, beta_len>::Identity();
            F.block<gamma_len, gamma_len>(beta_index, gamma_index) = -last_Rz * convert::cross_matrix<double>(hat_acc - last_ba);
            F.block<ba_len, ba_len>(beta_index, ba_index) = -last_Rz;
            F.block<gamma_len, gamma_len>(gamma_index, gamma_index) = -convert::cross_matrix<double>(hat_gyro - last_ba);
            F.block<bw_len, bw_len>(gamma_index, bw_index) = -Eigen::Matrix<double, bw_len, bw_len>::Identity();
            // 2.compute G
            Eigen::Matrix<double, all_status_len, all_noise_len> G = Eigen::Matrix<double, all_status_len, all_noise_len>::Zero();
            G.block<na_len, na_len>(beta_index, na_index) = -last_Rz;
            G.block<nw_len, nw_len>(gamma_index, nw_index) = -Eigen::Matrix<double, nw_len, nw_len>::Identity();
            G.block<nba_len, nba_len>(ba_index, nba_index) = Eigen::Matrix<double, nba_len, nba_len>::Identity();
            G.block<nbw_len, nbw_len>(bw_index, nbw_index) = Eigen::Matrix<double, nbw_len, nbw_len>::Identity();
            Eigen::Matrix<double, all_status_len, all_status_len> I = Eigen::Matrix<double, all_status_len, all_status_len>::Identity();
            // 3.update J
            F = (I + F * dt);
            J = F * J;
            // 4.update P

            P = F * P * F.transpose() + (G * dt) * imu_noise::get_imu_noise()->Q * (G * dt).transpose();
            Dt += dt;
        }
    };
} // namespace lvio_2d
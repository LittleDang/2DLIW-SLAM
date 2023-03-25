#pragma once
#include "factor/factor_common.h"

namespace lvio_2d
{
    struct marginalization_factor
    {
        Eigen::MatrixXd linearized_J;
        Eigen::VectorXd linearized_R;
        Eigen::VectorXd linearized_X;
        int n_world_point;
        marginalization_factor(const Eigen::VectorXd &X_,
                               const Eigen::MatrixXd &J_,
                               const Eigen::VectorXd &r_,
                               const int &n_world_point_)
        {
            linearized_J = J_;
            linearized_R = r_;
            linearized_X = X_;
            n_world_point = n_world_point_;
        }
        template <typename T>
        bool operator()(
            const T *const *states,
            T *res) const
        {
            Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic> X(15 + n_world_point * 3, 1);

            for (int i = 0; i < 3; i++) // p
                X(i) = states[0][i];

            for (int i = 0; i < 3; i++) // q
                X(i + 3) = states[1][i];

            for (int i = 0; i < 3; i++) // v
                X(i + 6) = states[2][i];

            for (int i = 0; i < 6; i++) // bs
                X(i + 9) = states[3][i];

            for (int i = 0; i < n_world_point; i++)
            {
                X(15 + i * 3 + 0) = states[4 + i][0];
                X(15 + i * 3 + 1) = states[4 + i][1];
                X(15 + i * 3 + 2) = states[4 + i][2];
            }

            Eigen::Map<Eigen::Matrix<T, Eigen::Dynamic, Eigen::Dynamic>> r_v(res, linearized_R.rows(), 1);

            r_v =/* linearized_R.cast<T>() +*/ linearized_J.cast<T>() * (X - linearized_X.template cast<T>());

            return true;
        }

        static ceres::CostFunction *Create(const Eigen::VectorXd &X_,
                                           const Eigen::MatrixXd &J_,
                                           const Eigen::VectorXd &r_,
                                           const int &n_world_point_)
        {

            auto cost_function = new ceres::DynamicAutoDiffCostFunction<marginalization_factor>(
                new marginalization_factor(X_, J_, r_, n_world_point_));

            cost_function->SetNumResiduals(r_.rows());

            cost_function->AddParameterBlock(3);
            cost_function->AddParameterBlock(3);
            cost_function->AddParameterBlock(3);
            cost_function->AddParameterBlock(6);
            for (int i = 0; i < n_world_point_; i++)
                cost_function->AddParameterBlock(3);
            return cost_function;
        }

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    };

} // namespace lvio_2d
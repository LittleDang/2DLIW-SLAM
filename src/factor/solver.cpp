#include "factor/solver.h"
#include "timerAndColor/timer.h"

std::tuple<Eigen::MatrixXd, Eigen::VectorXd> marginalization_matrix(const int &r_len, const Eigen::MatrixXd &J, const Eigen::VectorXd &R)
{
    int rows = J.rows();
    int cols = J.cols();

    Eigen::MatrixXd H;
    Eigen::VectorXd g;

    H = J.transpose() * J;
    g = -J.transpose() * R;

    int m_len = g.rows() - r_len;
    int m_index = 0;
    int r_index = m_len;

    Eigen::MatrixXd Hmm = H.block(m_index, m_index, m_len, m_len);

    Eigen::MatrixXd Hmm_inverse = Hmm.inverse();

    assert(BUG::maxtirx_is_valid(Hmm));
    assert(BUG::maxtirx_is_valid(Hmm_inverse));
    Eigen::MatrixXd Hmr = H.block(m_index, r_index, m_len, r_len);
    Eigen::MatrixXd Hrm = H.block(r_index, m_index, r_len, m_len);
    assert(Hmr == Hrm.transpose());

    assert(BUG::maxtirx_is_valid(Hmr));
    assert(BUG::maxtirx_is_valid(Hrm));
    Eigen::MatrixXd Hrr = H.block(r_index, r_index, r_len, r_len);

    Eigen::MatrixXd gm = g.block(m_index, 0, m_len, 1);
    Eigen::MatrixXd gr = g.block(r_index, 0, r_len, 1);

    Eigen::MatrixXd Delta_H = Hrr - Hrm * Hmm_inverse * Hmr;
    Eigen::VectorXd Delta_g = gr - Hrm * Hmm_inverse * gm;

    return {Delta_H, Delta_g};
}
namespace lvio_2d
{
    solver::solver()
    {
        has_linearized_block = false;
        linearized_world_ids.clear();
        linearized_world_points.clear();
    }

    void solver::do_init_solve(std::deque<frame_info::ptr> &frame_infos,
                               feature_manger &feature_infos,
                               bool enable_camera_factor)
    {
        ceres::LossFunction *loss_function = nullptr;
        ceres::LocalParameterization *so3_parameterization =
            factor::so3_parameterization::Create();
        ceres::Problem problem;

        // camera factor
        if (enable_camera_factor)
        {
            auto &features = feature_infos.get_feature_infos();
            for (auto &[id, iter] : features)
            {
                if (!iter->has_ready_for_opt)
                    return;
                for (int i = 0; i < iter->frame_indexs.size(); i++)
                {

                    assert(iter->frame_indexs[i] > -1);
                    assert(iter->frame_indexs[i] < frame_infos.size());

                    ceres::CostFunction *camera_cost_function =
                        camera_factor::Create(iter->camera_points[i]);

                    problem.AddResidualBlock(
                        camera_cost_function, loss_function,
                        frame_infos[iter->frame_indexs[i]]->p.data(), frame_infos[iter->frame_indexs[i]]->q.data(),
                        iter->world_point.data());

                    problem.SetParameterization(frame_infos[iter->frame_indexs[i]]->q.data(), so3_parameterization);
                }
            }
        }

        // laser factor
        for (int i = 0; i < frame_infos.size(); i++)
        {
            if (frame_infos[i]->type == frame_info::laser)
            {
                if (frame_infos[i]->laser_match_ptr)
                {
                    int index1 = 0;
                    int index2 = i;
                    for (int j = 0; j < frame_infos[i]->laser_match_ptr->lines1.size(); j++)
                    {
                        ceres::CostFunction *laser_cost_function = laser_factor::Create(
                            frame_infos[i]->laser_match_ptr->lines1[j]->p1,
                            frame_infos[i]->laser_match_ptr->lines1[j]->p2,
                            frame_infos[i]->laser_match_ptr->lines2[j]->p1,
                            frame_infos[i]->laser_match_ptr->lines2[j]->p2);

                        problem.AddResidualBlock(
                            laser_cost_function, loss_function,
                            frame_infos[index1]->p.data(), frame_infos[index1]->q.data(),
                            frame_infos[index2]->p.data(), frame_infos[index2]->q.data());

                        problem.SetParameterization(frame_infos[index1]->q.data(), so3_parameterization);
                        problem.SetParameterization(frame_infos[index2]->q.data(), so3_parameterization);
                    }
                }
            }
        }

        // imu_factor
        for (int i = 1; i < frame_infos.size(); i++)
        {
            ceres::CostFunction *imu_cost_function = imu_factor::Create(frame_infos[i]->imu_observation_reslut);
            problem.AddResidualBlock(
                imu_cost_function, loss_function,
                frame_infos[i - 1]->p.data(), frame_infos[i - 1]->q.data(), frame_infos[i - 1]->v.data(), frame_infos[i - 1]->bs.data(),
                frame_infos[i]->p.data(), frame_infos[i]->q.data(), frame_infos[i]->v.data(), frame_infos[i]->bs.data());

            problem.SetParameterization(frame_infos[i - 1]->q.data(), so3_parameterization);
            problem.SetParameterization(frame_infos[i]->q.data(), so3_parameterization);
        }

        // wheel_factor
        for (int i = 1; i < frame_infos.size(); i++)
        {
            ceres::CostFunction *wheel_cost_function = wheel_odom_factor::Create(frame_infos[i]->wheel_observation_reslut);
            problem.AddResidualBlock(
                wheel_cost_function, loss_function,
                frame_infos[i - 1]->p.data(), frame_infos[i - 1]->q.data(),
                frame_infos[i]->p.data(), frame_infos[i]->q.data());
            problem.SetParameterization(frame_infos[i - 1]->q.data(), so3_parameterization);
            problem.SetParameterization(frame_infos[i]->q.data(), so3_parameterization);
        }

        // ground factor

        for (int i = 0; i < frame_infos.size(); i++)
        {
            for (int j = 0; j < frame_infos.size(); j++)
            {
                ceres::CostFunction *ground_p_cost_function = ground_factor_p::Create();
                problem.AddResidualBlock(
                    ground_p_cost_function, loss_function,
                    frame_infos[j]->p.data(),
                    frame_infos[j]->q.data());

                ceres::CostFunction *ground_q_cost_function = ground_factor_q::Create();
                problem.AddResidualBlock(
                    ground_q_cost_function, loss_function,
                    frame_infos[j]->p.data(),
                    frame_infos[j]->q.data());
                problem.SetParameterization(frame_infos[j]->q.data(), so3_parameterization);
            }
        }

        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_SCHUR;
        options.use_nonmonotonic_steps = false;
        options.minimizer_progress_to_stdout = false;

        ceres::Solver::Summary summary;

        ceres::Solve(options, &problem, &summary);
    }

    void solver::init_solve(std::deque<frame_info::ptr> &frame_infos,
                            feature_manger &feature_infos)
    {
        do_init_solve(frame_infos, feature_infos, true);

        // 修正laser_match的p q
        for (int i = 0; i < frame_infos.size(); i++)
        {
            if (frame_infos[i]->type == frame_info::laser)
            {
                if (frame_infos[i]->laser_match_ptr)
                {
                    frame_infos[i]->laser_match_ptr->p1 = frame_infos[0]->p;
                    frame_infos[i]->laser_match_ptr->q1 = frame_infos[0]->q;

                    frame_infos[i]->laser_match_ptr->p2 = frame_infos[i]->p;
                    frame_infos[i]->laser_match_ptr->q2 = frame_infos[i]->q;
                }
            }
        }
        std::cout << "p: " << frame_infos.back()->p.transpose() << std::endl;
        std::cout << "q: " << frame_infos.back()->q.transpose() << std::endl;
        std::cout << "v: " << frame_infos.back()->v.transpose() << std::endl;
        std::cout << "bs: " << frame_infos.back()->bs.transpose() << std::endl;
    }

    void solver::clac_prior_J(std::deque<frame_info::ptr> &frame_infos,
                              feature_manger &feature_infos)
    {
        int last_prior_index = frame_infos.size() - 2;
        int r_index = 0;
        auto &all_features = feature_infos.get_feature_infos();
        int n_linearized_world_point_size = linearized_world_ids.size();
        auto frame_ptr = frame_infos[last_prior_index];

        marginalization_factor *functor = new marginalization_factor(linearized_X,
                                                                     linearized_jacobians,
                                                                     linearized_residuals,
                                                                     n_linearized_world_point_size);

        std::vector<double *> parametrs = {frame_ptr->p.data(),
                                           frame_ptr->q.data(),
                                           frame_ptr->v.data(),
                                           frame_ptr->bs.data()};
        std::vector<int> n_block = {3, 3, 3, 6};

        for (int i = 0; i < n_linearized_world_point_size; i++)
        {
            auto iter = all_features.find(linearized_world_ids[i]);
            if (iter != all_features.end())
                parametrs.push_back(iter->second->world_point.data());
            else
                parametrs.push_back(linearized_world_points[i].data());
            n_block.push_back(3);
        }
        Eigen::VectorXd res(linearized_residuals.size());
        std::vector<auto_diff::rMatrix> jacobians;
        auto_diff::compute_res_and_jacobi_dynamic(functor, parametrs, n_block, res, linearized_residuals.size(), jacobians);

        for (int l = 0; l < parametrs.size(); l++)
            assert(BUG::maxtirx_is_valid(jacobians[l]));
        assert(BUG::maxtirx_is_valid(res));

        J.block(r_index, all_status_block_indexs[last_prior_index].p,
                linearized_residuals.size(), n_block[0]) = jacobians[0];

        J.block(r_index, all_status_block_indexs[last_prior_index].q,
                linearized_residuals.size(), n_block[1]) = jacobians[1];

        J.block(r_index, all_status_block_indexs[last_prior_index].v,
                linearized_residuals.size(), n_block[2]) = jacobians[2];

        J.block(r_index, all_status_block_indexs[last_prior_index].bs,
                linearized_residuals.size(), n_block[3]) = jacobians[3];

        for (int j = 0; j < linearized_world_ids.size(); j++)
        {
            auto iter = all_features.find(linearized_world_ids[j]);
            if (iter != all_features.end())
                J.block(r_index, world_point_indexs[linearized_world_ids[j]],
                        linearized_residuals.size(), 3) = jacobians[4 + j];
        }

        R.block(r_index, 0, linearized_residuals.size(), 1) = res;
    }

    void solver::marginalization(std::deque<frame_info::ptr> &frame_infos, feature_manger &feature_infos)
    {
        if (PARAM(fast_mode))
            return;

        //  按照frame的顺序来
        //  X顺序 |other world point | [p q vbs m] | lastest camera world point
        //  r顺序 [laser_res | imu_res　| wheel_res | ground res ] | all camera res

        // 计算各种下标信息
        auto &all_features = feature_infos.get_feature_infos();
        auto &lastest_frame_features = feature_infos.get_lastest_frame_features();

        int n_all_features = all_features.size();
        int n_lastest_features = lastest_frame_features.size();
        int n = frame_infos.size();

        // g_dir n*[status + m] world point
        int all_X_size = n * (15);
        int all_res_size = 0;

        all_status_block_indexs.resize(n);

        int current_X_index = 0;
        int current_res_index = 0;
        if (has_linearized_block)
        {
            current_res_index += linearized_X.size();
        }
        for (int i = 0; i < n; i++)
        {
            // laser
            all_status_block_indexs[i].laser_res_index = current_res_index;
            int n_laser_match = 0;
            if (frame_infos[i]->type == frame_info::laser && frame_infos[i]->laser_match_ptr)
                n_laser_match = frame_infos[i]->laser_match_ptr->lines1.size();
            current_res_index += n_laser_match * 2;

            all_status_block_indexs[i].imu_res_index = current_res_index;
            if (i > 0) // imu
                current_res_index += 15;

            all_status_block_indexs[i].wheel_res_index = current_res_index;
            if (i > 0) // wheel
                current_res_index += 3;

            // ground
            all_status_block_indexs[i].ground_res_index = current_res_index;
            current_res_index += n * 2;
        }

        world_point_indexs.clear();
        camera_res_index = current_res_index;
        for (auto &[id, feature_ptr] : all_features)
        {
            if (!feature_ptr->has_estimate)
                continue;
            all_X_size += 3;
            current_res_index += feature_ptr->frame_indexs.size() * 2;
            bool is_lastest_point = false;
            for (int i = 0; i < n_lastest_features; i++)
            {
                if (lastest_frame_features[i] == feature_ptr)
                {
                    is_lastest_point = true;
                    break;
                }
            }
            if (!is_lastest_point)
            {
                world_point_indexs[id] = current_X_index;
                current_X_index += 3;
            }
        }

        for (int i = 0; i < n; i++)
        {
            all_status_block_indexs[i].p = current_X_index;
            current_X_index += 3;
            all_status_block_indexs[i].q = current_X_index;
            current_X_index += 3;
            all_status_block_indexs[i].v = current_X_index;
            current_X_index += 3;
            all_status_block_indexs[i].bs = current_X_index;
            current_X_index += 6;
        }
        std::vector<long long> tmp_linearized_world_ids;
        std::vector<Eigen::Vector3d> tmp_linearized_world_points;

        for (int i = 0; i < n_lastest_features; i++)
        {
            if (!lastest_frame_features[i]->has_estimate)
                continue;
            world_point_indexs[lastest_frame_features[i]->id] = current_X_index;
            current_X_index += 3;
            tmp_linearized_world_ids.push_back(lastest_frame_features[i]->id);
            tmp_linearized_world_points.push_back(lastest_frame_features[i]->world_point);
        }

        all_res_size = current_res_index;

        R = Eigen::VectorXd::Zero(all_res_size, 1);
        J = Eigen::MatrixXd::Zero(all_res_size, all_X_size);

        assert(current_X_index == all_X_size);

      

        // 先把参数打包成块

        if (has_linearized_block)
        {
            clac_prior_J(frame_infos, feature_infos);
        }
        linearized_world_ids = std::move(tmp_linearized_world_ids);
        linearized_world_points = std::move(tmp_linearized_world_points);

        for (int i = 0; i < n; i++)
        {

            clac_frame_J(frame_infos, i);
        }

        clac_camera_J(frame_infos, feature_infos);

        assert(BUG::maxtirx_is_valid(J));
        assert(BUG::maxtirx_is_valid(R));

        auto [Delta_H, Delta_g] = marginalization_matrix(15 + linearized_world_ids.size() * 3, J, R);

        assert(BUG::maxtirx_is_valid(Delta_H));
        assert(BUG::maxtirx_is_valid(Delta_g));

        const double eps = 1e-8;

        Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> saes2(Delta_H);
        Eigen::VectorXd S = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array(), 0));
        Eigen::VectorXd S_inv = Eigen::VectorXd((saes2.eigenvalues().array() > eps).select(saes2.eigenvalues().array().inverse(), 0));

        Eigen::VectorXd S_sqrt = S.cwiseSqrt();
        Eigen::VectorXd S_inv_sqrt = S_inv.cwiseSqrt();

        linearized_jacobians = S_sqrt.asDiagonal() * saes2.eigenvectors().transpose();

        frame_infos.back()->sqrt_H = linearized_jacobians.block<6, 6>(0, 0);
        linearized_residuals = -(S_inv_sqrt.asDiagonal() * saes2.eigenvectors().transpose() * Delta_g);

        assert(BUG::maxtirx_is_valid(linearized_jacobians));
        assert(BUG::maxtirx_is_valid(linearized_residuals));

        auto frame_ptr = frame_infos.back();

        linearized_X = Eigen::VectorXd::Zero(15 + linearized_world_ids.size() * 3, 1);

        linearized_X(0) = frame_ptr->p(0);
        linearized_X(1) = frame_ptr->p(1);
        linearized_X(2) = frame_ptr->p(2);

        linearized_X(3) = frame_ptr->q(0);
        linearized_X(4) = frame_ptr->q(1);
        linearized_X(5) = frame_ptr->q(2);

        linearized_X(6) = frame_ptr->v(0);
        linearized_X(7) = frame_ptr->v(1);
        linearized_X(8) = frame_ptr->v(2);

        linearized_X(9) = frame_ptr->bs(0);
        linearized_X(10) = frame_ptr->bs(1);
        linearized_X(11) = frame_ptr->bs(2);
        linearized_X(12) = frame_ptr->bs(3);
        linearized_X(13) = frame_ptr->bs(4);
        linearized_X(14) = frame_ptr->bs(5);

        int j = 0;
        for (int i = 0; i < n_lastest_features; i++)
        {
            if (!lastest_frame_features[i]->has_estimate)
                continue;
            linearized_X(15 + j * 3 + 0) = lastest_frame_features[i]->world_point(0);
            linearized_X(15 + j * 3 + 1) = lastest_frame_features[i]->world_point(1);
            linearized_X(15 + j * 3 + 2) = lastest_frame_features[i]->world_point(2);
            j++;
        }
        assert(linearized_X.rows() == linearized_residuals.rows());
        has_linearized_block = true;
    }
    void solver::clac_frame_J(std::deque<frame_info::ptr> &frame_infos,
                              int index)
    {
        int r_index = all_status_block_indexs[index].laser_res_index;
        // laser factor
        if (frame_infos[index]->type == frame_info::laser)
        {
            if (frame_infos[index]->laser_match_ptr)
            {

                for (int j = 0; j < frame_infos[index]->laser_match_ptr->lines1.size(); j++)
                {
                    laser_factor *functor = new laser_factor(frame_infos[index]->laser_match_ptr->lines1[j]->p1,
                                                             frame_infos[index]->laser_match_ptr->lines1[j]->p2,
                                                             frame_infos[index]->laser_match_ptr->lines2[j]->p1,
                                                             frame_infos[index]->laser_match_ptr->lines2[j]->p2);
                    std::vector<double *> parametrs = {frame_infos[index]->laser_match_ptr->p1.data(),
                                                       frame_infos[index]->laser_match_ptr->q1.data(),
                                                       frame_infos[index]->p.data(),
                                                       frame_infos[index]->q.data()};
                    Eigen::Matrix<double, 2, 1> res;
                    std::vector<auto_diff::rMatrix> jacobians;
                    auto_diff::compute_res_and_jacobi<laser_factor, 2, 3, 3, 3, 3>(functor, parametrs, res, jacobians);
                    for (int l = 0; l < 4; l++)
                        assert(BUG::maxtirx_is_valid(jacobians[l]));
                    assert(BUG::maxtirx_is_valid(res));


                    J.template block<2, alpha_len>(r_index, all_status_block_indexs[index].p) = jacobians[2];
                    J.template block<2, gamma_len>(r_index, all_status_block_indexs[index].q) = jacobians[3];

                    R.template block<2, 1>(r_index, 0) = res;
                    r_index += 2;
                }
            }
        }
        assert(r_index == all_status_block_indexs[index].imu_res_index);
        // imu_factor

        if (index > 0)
        {
            imu_factor *functor = new imu_factor(frame_infos[index]->imu_observation_reslut);
            std::vector<double *> parametrs = {frame_infos[index - 1]->p.data(),
                                               frame_infos[index - 1]->q.data(),
                                               frame_infos[index - 1]->v.data(),
                                               frame_infos[index - 1]->bs.data(),
                                               frame_infos[index]->p.data(),
                                               frame_infos[index]->q.data(),
                                               frame_infos[index]->v.data(),
                                               frame_infos[index]->bs.data()};
            Eigen::Matrix<double, all_status_len, 1> res;
            std::vector<auto_diff::rMatrix> jacobians;
            auto_diff::compute_res_and_jacobi<imu_factor, all_status_len, alpha_len, gamma_len, beta_len, ba_len + bw_len,
                                              alpha_len, gamma_len, beta_len, ba_len + bw_len>(functor, parametrs, res, jacobians);
            for (int l = 0; l < 8; l++)
                assert(BUG::maxtirx_is_valid(jacobians[l]));
            assert(BUG::maxtirx_is_valid(res));

            J.template block<15, alpha_len>(r_index, all_status_block_indexs[index - 1].p) = jacobians[0];
            J.template block<15, gamma_len>(r_index, all_status_block_indexs[index - 1].q) = jacobians[1];
            J.template block<15, 3>(r_index, all_status_block_indexs[index - 1].v) = jacobians[2];
            J.template block<15, 6>(r_index, all_status_block_indexs[index - 1].bs) = jacobians[3];

            J.template block<15, alpha_len>(r_index, all_status_block_indexs[index].p) = jacobians[4];
            J.template block<15, gamma_len>(r_index, all_status_block_indexs[index].q) = jacobians[5];
            J.template block<15, 3>(r_index, all_status_block_indexs[index].v) = jacobians[6];
            J.template block<15, 6>(r_index, all_status_block_indexs[index].bs) = jacobians[7];

            R.template block<15, 1>(r_index, 0) = res;
            r_index += 15;
        }
        assert(r_index == all_status_block_indexs[index].wheel_res_index);
        // wheel_factor

        if (index > 0)
        {
            wheel_odom_factor *functor = new wheel_odom_factor(frame_infos[index]->wheel_observation_reslut);
            std::vector<double *> parametrs = {frame_infos[index - 1]->p.data(),
                                               frame_infos[index - 1]->q.data(),
                                               frame_infos[index]->p.data(),
                                               frame_infos[index]->q.data()};

            Eigen::Matrix<double, 3, 1> res;
            std::vector<auto_diff::rMatrix> jacobians;
            auto_diff::compute_res_and_jacobi<wheel_odom_factor, 3, 3, 3, 3, 3>(functor, parametrs, res, jacobians);
            for (int l = 0; l < 4; l++)
                assert(BUG::maxtirx_is_valid(jacobians[l]));
            assert(BUG::maxtirx_is_valid(res));

            J.template block<3, alpha_len>(r_index, all_status_block_indexs[index - 1].p) = jacobians[0];
            J.template block<3, gamma_len>(r_index, all_status_block_indexs[index - 1].q) = jacobians[1];

            J.template block<3, alpha_len>(r_index, all_status_block_indexs[index].p) = jacobians[2];
            J.template block<3, gamma_len>(r_index, all_status_block_indexs[index].q) = jacobians[3];

            R.template block<3, 1>(r_index, 0) = res;

            r_index += 3;
        }
        assert(r_index == all_status_block_indexs[index].ground_res_index);

        // ground factor

        for (int j = 0; j < frame_infos.size(); j++)
        {
            // p
            {
                ground_factor_p *functor = new ground_factor_p;
                std::vector<double *> parametrs = {frame_infos[j]->p.data(),
                                                   frame_infos[j]->q.data()};

                Eigen::Matrix<double, 1, 1> res;
                std::vector<auto_diff::rMatrix> jacobians;
                auto_diff::compute_res_and_jacobi<ground_factor_p, 1, 3, 3>(functor, parametrs, res, jacobians);
                for (int l = 0; l < 2; l++)
                    assert(BUG::maxtirx_is_valid(jacobians[l]));
                assert(BUG::maxtirx_is_valid(res));

                J.template block<1, alpha_len>(r_index, all_status_block_indexs[j].p) = jacobians[0];
                J.template block<1, gamma_len>(r_index, all_status_block_indexs[j].q) = jacobians[1];

                R.template block<1, 1>(r_index, 0) = res;
            }
            r_index++;
            // q
            {
                ground_factor_q *functor = new ground_factor_q;
                std::vector<double *> parametrs = {frame_infos[j]->p.data(),
                                                   frame_infos[j]->q.data()};

                Eigen::Matrix<double, 1, 1> res;
                std::vector<auto_diff::rMatrix> jacobians;
                auto_diff::compute_res_and_jacobi<ground_factor_q, 1, 3, 3>(functor, parametrs, res, jacobians);
                for (int l = 0; l < 2; l++)
                    assert(BUG::maxtirx_is_valid(jacobians[l]));
                assert(BUG::maxtirx_is_valid(res));

                J.template block<1, alpha_len>(r_index, all_status_block_indexs[j].p) = jacobians[0];
                J.template block<1, gamma_len>(r_index, all_status_block_indexs[j].q) = jacobians[1];

                R.template block<1, 1>(r_index, 0) = res;
            }
            r_index++;
        }

        if (index < frame_infos.size() - 1)
            assert(r_index == all_status_block_indexs[index + 1].laser_res_index);
    }
    void solver::clac_camera_J(std::deque<frame_info::ptr> &frame_infos,
                               feature_manger &feature_infos)
    {
        auto &features = feature_infos.get_feature_infos();
        int r_index = camera_res_index;
        for (auto &[id, iter] : features)
        {
            if (!iter->has_estimate)
                continue;
            for (int i = 0; i < iter->frame_indexs.size(); i++)
            {

                assert(iter->frame_indexs[i] > -1);
                assert(iter->frame_indexs[i] < frame_infos.size());

                camera_factor *functor = new camera_factor(iter->camera_points[i]);
                std::vector<double *> parametrs = {frame_infos[iter->frame_indexs[i]]->p.data(),
                                                   frame_infos[iter->frame_indexs[i]]->q.data(),
                                                   iter->world_point.data()};

                Eigen::Matrix<double, 2, 1> res;
                std::vector<auto_diff::rMatrix> jacobians;
                auto_diff::compute_res_and_jacobi<camera_factor, 2, 3, 3, 3>(functor, parametrs, res, jacobians);

                for (int l = 0; l < 3; l++)
                    assert(BUG::maxtirx_is_valid(jacobians[l]));
                assert(BUG::maxtirx_is_valid(res));
                J.template block<2, 3>(r_index, all_status_block_indexs[iter->frame_indexs[i]].p) = jacobians[0];
                J.template block<2, 3>(r_index, all_status_block_indexs[iter->frame_indexs[i]].q) = jacobians[1];
                J.template block<2, 3>(r_index, world_point_indexs[iter->id]) = jacobians[2];
                R.template block<2, 1>(r_index, 0) = res;
                r_index += 2;
            }
            iter->linearized_times++;
            if (iter->linearized_times >= 3)
                iter->has_ready_for_opt = true;
        }
        assert(r_index == R.rows());
    }

    void solver::solve(std::deque<frame_info::ptr> &frame_infos,
                       feature_manger &feature_infos)
    {

        ceres::LossFunction *loss_function = nullptr;
        ceres::LocalParameterization *so3_parameterization =
            factor::so3_parameterization::Create();
        ceres::Problem problem;

        // camera factor
        auto &features = feature_infos.get_lastest_frame_features();
        for (auto iter : features)
        {
            if (!iter->has_ready_for_opt)
                continue;
            for (int i = 0; i < iter->frame_indexs.size(); i++)
            {

                if (iter->frame_indexs[i] < 0)
                {
                    std::cout << "error" << std::endl;
                    std::cout << iter->frame_indexs[i] << std::endl;
                    continue;
                }
                ceres::CostFunction *camera_cost_function =
                    camera_factor::Create(iter->camera_points[i]);

                problem.AddResidualBlock(
                    camera_cost_function, loss_function,
                    frame_infos[iter->frame_indexs[i]]->p.data(),
                    frame_infos[iter->frame_indexs[i]]->q.data(),
                    iter->world_point.data());

                problem.SetParameterization(frame_infos[iter->frame_indexs[i]]->q.data(), so3_parameterization);
            }
        }

        // laser factor
        for (int i = frame_infos.size() - 1; i < frame_infos.size(); i++)
        {
            if (frame_infos[i]->type == frame_info::laser)
            {
                if (frame_infos[i]->laser_match_ptr)
                {
                    for (int j = 0; j < frame_infos[i]->laser_match_ptr->lines1.size(); j++)
                    {
                        ceres::CostFunction *laser_cost_function = laser_factor::Create(
                            frame_infos[i]->laser_match_ptr->lines1[j]->p1,
                            frame_infos[i]->laser_match_ptr->lines1[j]->p2,
                            frame_infos[i]->laser_match_ptr->lines2[j]->p1,
                            frame_infos[i]->laser_match_ptr->lines2[j]->p2);

                        problem.AddResidualBlock(
                            laser_cost_function, loss_function,
                            frame_infos[i]->laser_match_ptr->p1.data(),
                            frame_infos[i]->laser_match_ptr->q1.data(),
                            frame_infos[i]->p.data(),
                            frame_infos[i]->q.data());

                        problem.SetParameterBlockConstant(frame_infos[i]->laser_match_ptr->p1.data());
                        problem.SetParameterBlockConstant(frame_infos[i]->laser_match_ptr->q1.data());
                        problem.SetParameterization(frame_infos[i]->laser_match_ptr->q1.data(), so3_parameterization);

                        problem.SetParameterization(frame_infos[i]->q.data(), so3_parameterization);
                    }
                }
            }
        }

        // imu_factor
        for (int i = 1; i < frame_infos.size(); i++)
        {
            ceres::CostFunction *imu_cost_function = imu_factor::Create(frame_infos[i]->imu_observation_reslut);
            problem.AddResidualBlock(
                imu_cost_function, loss_function,
                frame_infos[i - 1]->p.data(), frame_infos[i - 1]->q.data(), frame_infos[i - 1]->v.data(), frame_infos[i - 1]->bs.data(),
                frame_infos[i]->p.data(), frame_infos[i]->q.data(), frame_infos[i]->v.data(), frame_infos[i]->bs.data());

            problem.SetParameterization(frame_infos[i - 1]->q.data(), so3_parameterization);
            problem.SetParameterization(frame_infos[i]->q.data(), so3_parameterization);
        }

        // wheel_factor
        for (int i = 1; i < frame_infos.size(); i++)
        {
            ceres::CostFunction *wheel_cost_function = wheel_odom_factor::Create(frame_infos[i]->wheel_observation_reslut);
            problem.AddResidualBlock(
                wheel_cost_function, loss_function,
                frame_infos[i - 1]->p.data(), frame_infos[i - 1]->q.data(),
                frame_infos[i]->p.data(), frame_infos[i]->q.data());
            problem.SetParameterization(frame_infos[i - 1]->q.data(), so3_parameterization);
            problem.SetParameterization(frame_infos[i]->q.data(), so3_parameterization);
        }

        // ground factor

        for (int i = 0; i < frame_infos.size(); i++)
        {
            for (int j = 0; j < frame_infos.size(); j++)
            {
                ceres::CostFunction *ground_p_cost_function = ground_factor_p::Create();
                problem.AddResidualBlock(
                    ground_p_cost_function, loss_function,
                    frame_infos[j]->p.data(),
                    frame_infos[j]->q.data());

                ceres::CostFunction *ground_q_cost_function = ground_factor_q::Create();
                problem.AddResidualBlock(
                    ground_q_cost_function, loss_function,
                    frame_infos[j]->p.data(),
                    frame_infos[j]->q.data());
            }
        }
        if (!PARAM(fast_mode))
            if (has_linearized_block)
            {

                std::vector<double *> parameter_blocks;
                parameter_blocks.clear();
                auto frame_ptr = frame_infos[frame_infos.size() - 2];
                auto &all_features = feature_infos.get_feature_infos();
                // marginalization_factor
                auto marginalization_cost_function = marginalization_factor::Create(
                    linearized_X, linearized_jacobians, linearized_residuals, linearized_world_ids.size());

                parameter_blocks.push_back(frame_ptr->p.data());
                parameter_blocks.push_back(frame_ptr->q.data());
                parameter_blocks.push_back(frame_ptr->v.data());
                parameter_blocks.push_back(frame_ptr->bs.data());

                std::vector<double *> const_block;
                for (int i = 0; i < linearized_world_ids.size(); i++)
                {
                    auto iter = all_features.find(linearized_world_ids[i]);
                    if (iter != all_features.end())
                    {
                        parameter_blocks.push_back(iter->second->world_point.data());
                        if (!iter->second->has_ready_for_opt)
                            const_block.push_back(iter->second->world_point.data());
                    }
                    else
                    {
                        parameter_blocks.push_back(linearized_world_points[i].data());
                        const_block.push_back(linearized_world_points[i].data());
                    }
                }
                problem.AddResidualBlock(
                    marginalization_cost_function, loss_function, parameter_blocks);

                problem.SetParameterization(frame_ptr->q.data(), so3_parameterization);
                for (int i = 0; i < const_block.size(); i++)
                {
                    problem.SetParameterBlockConstant(const_block[i]);
                }
            }

        for (int i = 0; i < frame_infos.size() - 1; i++)
        {
            problem.SetParameterBlockConstant(frame_infos[i]->p.data());
            problem.SetParameterBlockConstant(frame_infos[i]->q.data());
            if (PARAM(fast_mode))
                problem.SetParameterBlockConstant(frame_infos[i]->bs.data());
            // problem.SetParameterBlockConstant(frame_infos[i]->v.data());
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.minimizer_progress_to_stdout = false;
        // options.num_threads = 2;
        ceres::Solver::Summary summary;
        if (PARAM(fast_mode))
            options.max_num_iterations = 10;
        ceres::Solve(options, &problem, &summary);

        for (int i = frame_infos.size() - 1; i < frame_infos.size(); i++)
        {
            if (frame_infos[i]->type == frame_info::laser)
            {
                if (frame_infos[i]->laser_match_ptr)
                {
                    frame_infos[i]->laser_match_ptr->p2 = frame_infos[i]->p;
                    frame_infos[i]->laser_match_ptr->q2 = frame_infos[i]->q;
                }
            }
        }

        // std::cout << "p: " << frame_infos.back()->p.transpose() << std::endl;
        // std::cout << "q: " << frame_infos.back()->q.transpose() << std::endl;
        // std::cout << "v: " << frame_infos.back()->v.transpose() << std::endl;
        // std::cout << "bs: " << frame_infos.back()->bs.transpose() << std::endl;
    }

} // namespace lvio_2d
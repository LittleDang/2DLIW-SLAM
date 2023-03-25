#include "trajectory/laser_manager.h"
#include "timerAndColor/timer.h"
constexpr double epsilo = 0.0008;

// helper fun
namespace lvio_2d
{
    Eigen::Vector3d project_to_line(const Eigen::Vector3d &p, const Eigen::Vector3d &start_point, const Eigen::Vector3d &end_point)
    {
        if ((end_point - start_point).norm() < epsilo)
            return p;
        Eigen::Vector3d se_unit_vec = (end_point - start_point).normalized();
        Eigen::Vector3d sp_vec = p - start_point;
        double line_project_norm = sp_vec.dot(se_unit_vec);
        Eigen::Vector3d ret_p = start_point + line_project_norm * se_unit_vec;
        return ret_p;
    }

    Eigen::Vector3d fit_line_by_least_square(const std::vector<Eigen::Vector3d> &points,
                                             int index1,
                                             int index2)
    {
        int num = index2 - index1 + 1;
        assert(num >= 3);
        Eigen::MatrixXd A(num, 3);
        for (int i = index1; i <= index2; i++)
        {
            A(i - index1, 0) = points[i](0);
            A(i - index1, 1) = points[i](1);
            A(i - index1, 2) = 1;
        }
        auto SVD = Eigen::JacobiSVD<Eigen::MatrixXd>(A, Eigen::ComputeThinU | Eigen::ComputeThinV);
        Eigen::Vector3d ret = SVD.matrixV().col(2);

        return ret;
    }
    std::tuple<double, Eigen::Vector3d> calc_angle_and_intersection(const line::ptr &l1, const line::ptr &l2)
    {
        if (l1 == l2)
        {
            ROS_ERROR("ERROR");
        }
        Eigen::Vector3d v1 = l1->p1 - l1->p2;
        Eigen::Vector3d v2 = l2->p1 - l2->p2;
        double angle = acos(v1.normalized().dot(v2.normalized()));
        Eigen::Matrix<double, 2, 2> A;
        Eigen::Vector2d b;
        A(0, 0) = l1->abc(0);
        A(0, 1) = l1->abc(1);
        A(1, 0) = l2->abc(0);
        A(1, 1) = l2->abc(1);
        b(0) = -l1->abc(2);
        b(1) = -l2->abc(2);
        Eigen::JacobiSVD<Eigen::Matrix<double, 2, 2>> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
        Eigen::Vector2d inter = svd.solve(b);
        Eigen::Vector3d inter3d;
        inter3d(0) = inter(0);
        inter3d(1) = inter(1);
        inter3d(2) = 0;
        return {angle, inter3d};
    }
    std::tuple<Eigen::Vector3d, Eigen::Vector3d, Eigen::Vector3d, double> create_line(const std::vector<Eigen::Vector3d> &points,
                                                                                      const Eigen::Vector3d &line_abc,
                                                                                      int index1,
                                                                                      int index2)
    {
        const Eigen::Vector3d &ret = line_abc;
        Eigen::Vector3d point1(0, 0, 0), point2(0, 0, 0);

        if (fabs(ret(1)) < 0.5) // b==0
        {
            point1(1) = 0;
            point1(0) = -ret(2) / ret(0);
            point2(1) = 1;
            point2(0) = (-ret(2) - ret(1)) / ret(0);
        }
        else
        {
            point1(0) = 0;
            point2(0) = 1;
            point1(1) = -ret(2) / ret(1);
            point2(1) = (-ret(2) - ret(0)) / ret(1);
        }
        double max_dis = 0;
        for (int i = index1; i <= index2; i++)
        {
            double tmp_error = e_laser::dis_from_line(points[i], point1, point2);
            if (tmp_error > max_dis)
                max_dis = tmp_error;
        }
        return {project_to_line(points[index1], point1, point2),
                project_to_line(points[index2], point1, point2),
                ret, max_dis};
    }

    bool is_continuous(const Eigen::Vector3d &point1, const Eigen::Vector3d &point2)
    {
        return (point1 - point2).norm() <= PARAM(line_continuous_threshold);
    }
    double clac_cos(const Eigen::Vector3d &point_j,
                    const Eigen::Vector3d &point_i,
                    const Eigen::Vector3d &point_k)
    {
        if ((point_i - point_j).norm() < epsilo)
            return -1;
        if ((point_j - point_k).norm() < epsilo)
            return -1;
        Eigen::Vector3d ji = (point_i - point_j).normalized();
        Eigen::Vector3d jk = (point_k - point_j).normalized();
        return (ji).dot(jk);
    }
    double clac_angle(const Eigen::Vector3d &point_j,
                      const Eigen::Vector3d &point_i,
                      const Eigen::Vector3d &point_k)
    {
        return acos(clac_cos(point_j, point_i, point_k));
    }
    double dis(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2)
    {
        return (p1 - p2).norm();
    }

} // namespace lvio_2d

// line scan
namespace lvio_2d
{
    line::line(const Eigen::Vector3d &p1_, const Eigen::Vector3d &p2_, const Eigen::Vector3d &abc_)

    {
        p1 = p1_;
        p2 = p2_;
        abc = abc_;
        len = (p1 - p2).norm();
    }

    void scan::add_line(const std::vector<Eigen::Vector3d> &points, const int &index1, const int &index2, bool add_concers)
    {
        if (index2 - index1 < 2)
        {
            return;
        }

        auto line_abc = fit_line_by_least_square(points, index1, index2);
        auto [p1, p2, abc, error] = create_line(points, line_abc, index1, index2);
        line::ptr l = std::make_shared<line>(p1, p2, abc);
        double len = (p1 - p2).norm();

        if (error > PARAM(line_max_dis))
            return;
        if (len < PARAM(line_min_len))
            return;
        if (add_concers)
        {
            for (int i = index1; i <= index2; i++)
            {
                auto [c, r] = xy_to_index(points[i](0), points[i](1));
                if (is_index_valid(r, c))
                {
                    if (line_map(r, c).empty())
                    {
                        line_map(r, c).push_back(l);
                        if (lines.empty() || lines.back() != l)
                            lines.push_back(l);
                    }
                    else if (l != line_map(r, c).back())
                    {
                        line_map(r, c).push_back(l);
                        if (lines.empty() || lines.back() != l)
                            lines.push_back(l);
                    }
                    else
                        continue;

                    if (line_map(r, c).size() == 2)
                    {
                        if (line_map(r, c)[0]->len > 0.1 &&
                            line_map(r, c)[1]->len > 0.1)
                        {
                            auto [angle, inter] = calc_angle_and_intersection(line_map(r, c)[0], line_map(r, c)[1]);
                            if (angle < convert::angle_to_rad(150) && angle > convert::angle_to_rad(30))
                            {
                                auto [concer_c, concer_r] = xy_to_index(inter(0), inter(1));
                                if (fabs(concer_r - r) <= 1 && fabs(concer_c - c) <= 1)
                                    concers.push_back(inter);
                            }
                        }
                    }
                }
            }
        }
        else
        {
            Eigen::Vector3d unit = (p2 - p1).normalized();
            for (double tr = 0; tr <= len; tr += 0.05)
            {
                Eigen::Vector3d tmp = p1 + unit * tr;
                auto [c, r] = xy_to_index(tmp(0), tmp(1));
                if (is_index_valid(r, c))
                {
                    if (line_map(r, c).empty() || l != line_map(r, c).back())
                    {
                        line_map(r, c).push_back(l);
                        if (lines.empty() || lines.back() != l)
                            lines.push_back(l);
                    }
                    // else if (line_map(r,c).front()->len < 0.1 && len > line_map(r,c).front()->len)
                    // {
                    //     line_map(r,c).front() = l;
                    // }
                }
            }
        }
    }
    void scan::add_line(const Eigen::Vector3d &p1, const Eigen::Vector3d &p2, bool add_concers)
    {
        std::vector<Eigen::Vector3d> fake_points;
        Eigen::Vector3d mid_point = (p2 + p1) / 2;
        fake_points.push_back(p1);
        fake_points.push_back(mid_point);
        fake_points.push_back(p2);
        add_line(fake_points, 0, 2, add_concers);
    }
} // namespace lvio_2d

namespace lvio_2d
{

    laser_manager::laser_manager()
    {
        double w_laser_each_scan = PARAM(w_laser_each_scan);
        double h_laser_each_scan = PARAM(h_laser_each_scan);
        double laser_resolution = PARAM(laser_resolution);
        w = w_laser_each_scan / laser_resolution + 1;
        h = h_laser_each_scan / laser_resolution + 1;
        resolution = laser_resolution;
        line_max_tolerance_angle =
            convert::angle_to_rad(PARAM(line_max_tolerance_angle));

        ref_submap_ptr = nullptr;
        spawnning_ref_submap_ptr = nullptr;
    }

    laser_match::ptr laser_manager::do_match(const scan::ptr &scan1, const scan::ptr &scan2, const Eigen::Vector3d &p1_, const Eigen::Vector3d &q1_,
                                             const Eigen::Vector3d &p2_, const Eigen::Vector3d &q2_, int kk)
    {
        Eigen::Isometry3d T_1_2 = (lie::make_tf(p1_, q1_) * PARAM(T_imu_to_laser)).inverse() *
                                  (lie::make_tf(p2_, q2_) * PARAM(T_imu_to_laser));
        laser_match::ptr ret(new laser_match);
        ret->p1 = p1_;
        ret->q1 = q1_;
        ret->p2 = p2_;
        ret->q2 = q2_;
        ret->scan2 = scan2;
        for (int i = 0; i < scan2->lines.size(); i++)
        {
            Eigen::Vector3d p1 = scan2->lines[i]->p1;
            Eigen::Vector3d p2 = scan2->lines[i]->p2;
            std::vector<line::ptr> tmp_lines;
            Eigen::Vector3d mid_p = (p1 + p2) / 2;

            {
                Eigen::Vector3d transform_mid_p = T_1_2 * mid_p;
                auto [c, r] =
                    scan1->xy_to_index(transform_mid_p(0), transform_mid_p(1));
                int a = 1 + kk;
                for (int dr = -a; dr <= a; dr++)
                {
                    for (int dc = -a; dc <= a; dc++)
                    {
                        int tmp_r = r + dr;
                        int tmp_c = c + dc;
                        if (scan1->is_index_valid(tmp_r, tmp_c))
                        {
                            tmp_lines.insert(tmp_lines.end(),
                                             scan1->line_map(tmp_r, tmp_c).begin(),
                                             scan1->line_map(tmp_r, tmp_c).end());
                        }
                    }
                }
            }

            if (tmp_lines.empty())
                continue;

            line::ptr best_match_line = nullptr;
            double best_angle = M_PI * 2;
            Eigen::Vector3d v2 =
                T_1_2 * scan2->lines[i]->p2 - T_1_2 * scan2->lines[i]->p1;

            for (int j = 0; j < tmp_lines.size(); j++)
            {
                Eigen::Vector3d v1 = tmp_lines[j]->p2 - tmp_lines[j]->p1;
                double angle = acos(std::fabs(v1.normalized().dot(v2.normalized())));
                if (angle < best_angle)
                {
                    best_match_line = tmp_lines[j];
                    best_angle = angle;
                }
            }
            if (convert::rad_to_angle(best_angle) > 10)
                continue;
            // double len1 = (best_match_line->p1 - best_match_line->p2).norm();
            // double len2 = (scan2->lines[i]->p1 - scan2->lines[i]->p2).norm();
            // double max_len = std::max(len1, len2);
            // double min_len = std::min(len1, len2);
            // if (min_len / max_len < 0.8)
            //     continue;

            ret->lines1.push_back(best_match_line);
            ret->lines2.push_back(scan2->lines[i]);
        }

        // 过滤ret的结果
        // 1.计算平均线和线之间的距离，过滤掉大于均值*k的线
        double aver_dis = 0;
        std::vector<double> diss(ret->lines1.size(), 0);
        for (int i = 0; i < ret->lines1.size(); i++)
        {
            Eigen::Vector3d p1 = T_1_2 * ret->lines2[i]->p1;
            Eigen::Vector3d p2 = T_1_2 * ret->lines2[i]->p2;

            double dis = 0.5 * (e_laser::dis_from_line(p1, ret->lines1[i]->p1, ret->lines1[i]->p2) +
                                e_laser::dis_from_line(p2, ret->lines1[i]->p1, ret->lines1[i]->p2));
            aver_dis += dis;
            diss[i] = dis;
        }
        aver_dis /= ret->lines1.size();

        laser_match::ptr ret2(new laser_match);
        ret2->p1 = p1_;
        ret2->q1 = q1_;
        ret2->p2 = p2_;
        ret2->q2 = q2_;
        ret2->scan2 = scan2;

        double k = 1.2;
        for (int i = 0; i < ret->lines1.size(); i++)
        {
            if (diss[i] < aver_dis * k)
            {
                ret2->lines1.push_back(ret->lines1[i]);
                ret2->lines2.push_back(ret->lines2[i]);
            }
        }

        return ret2;
    }

    scan::ptr laser_manager::spawn_scan(const sensor::laser::u_ptr &laser)
    {

        // lmicroTimer("spawn_scan");
        //  假设已经通过某种手段消除去了运动畸变
        double time = laser->times_ptr->front();
        std::vector<Eigen::Vector3d> &points = *(laser->points_ptr);
        scan::ptr current_scan = std::make_shared<scan>(w, h, resolution, time);
        if (PARAM(enable_laser_vis))
            current_scan->points = points;
        // lmicroTimer("add laser");
        std::vector<std::tuple<int, int>> line_start_end_indexs;
        {
            int start_index = 0;
            int end_index = points.size() - 1;
            for (int i = 1; i < points.size(); i++)
                if (!is_continuous(points[i - 1], points[i]))
                {
                    end_index = i - 1;
                    line_start_end_indexs.emplace_back(start_index, end_index);
                    start_index = i;
                }
            end_index = points.size() - 1;
            line_start_end_indexs.emplace_back(start_index, end_index);
        }
        int step = 3;
        std::vector<double> responses(points.size(), -1);

        for (const auto &[start_index, end_index] : line_start_end_indexs)
        {
            int num = end_index - start_index + 1;

            std::vector<int> maybe_end_points;
            for (int i = start_index + 1; i <= end_index - 1; i++)
            {
                responses[i] =
                    clac_cos(points[i], points[std::max(i - step, start_index)], points[std::min(i + step, end_index)]);
            }

            maybe_end_points.push_back(start_index);
            for (int i = start_index + 1; i <= end_index - 1; i++)
            {

                bool is_max = true;
                int begin_j = std::max(i - step, start_index + 1);
                int end_j = std::min(i + step, end_index - 1);
                for (int j = begin_j; j <= end_j; j++)
                    if (responses[j] >= responses[i] && j != i)
                    {
                        is_max = false;
                        break;
                    }
                if (is_max)
                {
                    maybe_end_points.push_back(i);
                    i += step;
                }
            }
            maybe_end_points.push_back(end_index);
            int last_end_index = 0;
            for (int i = 1; i < maybe_end_points.size() - 1; i++)
            {
                double angle = clac_angle(points[maybe_end_points[i]], points[maybe_end_points[last_end_index]], points[maybe_end_points[i + 1]]);
                if (std::abs(angle) < line_max_tolerance_angle)
                {
                    current_scan->add_line(points, maybe_end_points[last_end_index], maybe_end_points[i]);
                    last_end_index = i;
                }
            }
            current_scan->add_line(points, maybe_end_points[last_end_index], maybe_end_points.back());
        }
        return current_scan;
    }

    void laser_manager::add_scan(const scan::ptr &scan_ptr,
                                 const Eigen::Vector3d &current_p,
                                 const Eigen::Vector3d &current_q)
    {
        // lmicroTimer("add_scan");
        laser_submap::ptr submap_ptr(new laser_submap(scan_ptr, current_p, current_q));
        key_frame.push_back(submap_ptr);

        auto current_tf = lie::make_tf(current_p, current_q);
        if (ref_submap_ptr != nullptr)
        {
            auto [dp, dq] = lie::log_SE3<double>(last_add_tf.inverse() * current_tf);
            if (dp.norm() < PARAM(ref_motion_filter_p) && dq.norm() < PARAM(ref_motion_filter_q))
                return;
        }
        else // 初始化 ref_submap_ptr
        {
            scan::ptr tmp_scan = std::make_shared<scan>(w, h, resolution, 0);
            ref_submap_ptr = laser_submap::ptr(new laser_submap(tmp_scan, current_p, current_q));
            last_add_tf = current_tf;
            current_count = 1;
            for (int i = 0; i < scan_ptr->lines.size(); i++)
                ref_submap_ptr->scan_ptr->add_line(scan_ptr->lines[i]->p1, scan_ptr->lines[i]->p2, false);
            return;
        }

        for (int i = 0; i < scan_ptr->lines.size(); i++)
        {
            {
                Eigen::Isometry3d tf_ref = lie::make_tf(ref_submap_ptr->current_p, ref_submap_ptr->current_q);
                Eigen::Isometry3d tf_ref_current = tf_ref.inverse() * current_tf;
                Eigen::Isometry3d l_tf_ref_current = PARAM(T_imu_to_laser).inverse() * tf_ref_current * PARAM(T_imu_to_laser);
                Eigen::Vector3d p1 = l_tf_ref_current * scan_ptr->lines[i]->p1;
                Eigen::Vector3d p2 = l_tf_ref_current * scan_ptr->lines[i]->p2;
                ref_submap_ptr->scan_ptr->add_line(p1, p2, false);
            }
            if (spawnning_ref_submap_ptr)
            {
                Eigen::Isometry3d tf_spawnning_ref = lie::make_tf(spawnning_ref_submap_ptr->current_p, spawnning_ref_submap_ptr->current_q);
                Eigen::Isometry3d tf_spawnning_ref_current = tf_spawnning_ref.inverse() * current_tf;
                Eigen::Isometry3d l_tf_spawnning_ref_current = PARAM(T_imu_to_laser).inverse() * tf_spawnning_ref_current * PARAM(T_imu_to_laser);
                Eigen::Vector3d p1 = l_tf_spawnning_ref_current * scan_ptr->lines[i]->p1;
                Eigen::Vector3d p2 = l_tf_spawnning_ref_current * scan_ptr->lines[i]->p2;
                spawnning_ref_submap_ptr->scan_ptr->add_line(p1, p2, false);
            }
        }

        current_count++;

        if (spawnning_ref_submap_ptr == nullptr)
        {
            if (current_count == PARAM(ref_n_accumulation) / 2)
            {
                scan::ptr tmp_scan = std::make_shared<scan>(w, h, resolution, 0);
                spawnning_ref_submap_ptr = laser_submap::ptr(new laser_submap(tmp_scan, current_p, current_q));
                last_add_tf = lie::make_tf(current_p, current_q);
                for (int i = 0; i < scan_ptr->lines.size(); i++)
                    spawnning_ref_submap_ptr->scan_ptr->add_line(scan_ptr->lines[i]->p1, scan_ptr->lines[i]->p2, false);
            }
        }

        if (current_count == PARAM(ref_n_accumulation))
        {
            ref_submap_ptr = spawnning_ref_submap_ptr;
            scan::ptr tmp_scan = std::make_shared<scan>(w, h, resolution, 0);
            spawnning_ref_submap_ptr = laser_submap::ptr(new laser_submap(tmp_scan, current_p, current_q));
            last_add_tf = lie::make_tf(current_p, current_q);
            for (int i = 0; i < scan_ptr->lines.size(); i++)
                spawnning_ref_submap_ptr->scan_ptr->add_line(scan_ptr->lines[i]->p1, scan_ptr->lines[i]->p2, false);
            current_count = PARAM(ref_n_accumulation) / 2;
        }
        last_add_tf = current_tf;
    }

    laser_match::ptr laser_manager::match_with_front(const scan::ptr scan_ptr, const Eigen::Vector3d &current_p,
                                                     const Eigen::Vector3d &current_q)
    {

        if (key_frame.empty())
        {
            laser_match::ptr ret(new laser_match);
            ret->p1 = current_p;
            ret->p2 = current_p;
            ret->q1 = current_q;
            ret->q2 = current_q;
            ret->scan2 = scan_ptr;
            return ret;
        }
        return do_match(key_frame.front()->scan_ptr, scan_ptr, key_frame.front()->current_p,
                        key_frame.front()->current_q, current_p, current_q);
    }
    laser_match::ptr laser_manager::match_with_back(const scan::ptr scan_ptr, const Eigen::Vector3d &current_p,
                                                    const Eigen::Vector3d &current_q)
    {
        if (key_frame.empty())
        {
            laser_match::ptr ret(new laser_match);
            ret->p1 = current_p;
            ret->p2 = current_p;
            ret->q1 = current_q;
            ret->q2 = current_q;
            ret->scan2 = scan_ptr;
            return ret;
        }
        return do_match(key_frame.back()->scan_ptr, scan_ptr, key_frame.back()->current_p,
                        key_frame.back()->current_q, current_p, current_q);
    }
    laser_match::ptr laser_manager::match_with_ref(const scan::ptr scan_ptr, const Eigen::Vector3d &current_p,
                                                   const Eigen::Vector3d &current_q)
    {
        if (ref_submap_ptr == nullptr)
        {
            laser_match::ptr ret(new laser_match);
            ret->p1 = current_p;
            ret->p2 = current_p;
            ret->q1 = current_q;
            ret->q2 = current_q;
            ret->scan2 = scan_ptr;
            return ret;
        }
        return do_match(ref_submap_ptr->scan_ptr, scan_ptr, ref_submap_ptr->current_p,
                        ref_submap_ptr->current_q, current_p, current_q);
    }
    laser_submap::ptr laser_manager::pop_scan()
    {
        if (key_frame.empty())
            return nullptr;
        laser_submap::ptr ret = key_frame.front();
        ret->scan_ptr->line_map.destroy();
        // ret->scan_ptr->line_map.clear();
        // ret->scan_ptr->line_map.shrink_to_fit();

        key_frame.pop_front();
        return ret;
    }

    void laser_manager::clear_all_scan()
    {
        key_frame.clear();
        ref_submap_ptr = nullptr;
        spawnning_ref_submap_ptr = nullptr;
    }

} // namespace lvio_2d

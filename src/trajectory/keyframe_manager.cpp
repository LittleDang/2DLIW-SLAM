#include "trajectory/keyframe_manager.h"
#include "factor/edge_factor.h"
#include "factor/ground_factor.h"
#include "factor/laser_factor.h"
#include "factor/point_factor.h"
#include "trajectory/laser_manager.h"
#include <opencv2/opencv.hpp>
static lvio_2d::record *recorder;
static inline int CountOnes(uint64_t n)
{
    int count = 0;
    while (n)
    {
        ++count;
        n = n & (n - 1);
    }
    return count;
}
namespace lvio_2d
{
    des_i::des_i()
    {
        constexpr double max_len = 100;
        int n = (max_len / PARAM(d_res) + 1) / 64 + 1;
        quick_des = std::vector<std::uint64_t>(n, 0);
    }
    void des_i::set_1(int dij)
    {
        int m = dij / 64;
        int n = dij % 64;
        if (m >= quick_des.size())
            return;
        quick_des[m] |= (1ul << n);
    }

} // namespace lvio_2d
namespace lvio_2d
{
    static bool is_index_valid(const int &r, const int &c, const int &w, const int &h)
    {
        return r >= 0 && r < h && c >= 0 && c < w;
    }
    static std::tuple<int, int> xy_to_index(const double &x, const double &y, const int &w, const int &h)
    {
        return {x / PARAM(laser_resolution) + w / 2, y / PARAM(laser_resolution) + h / 2};
    }

    static cv::Point to_cv_Point(const Eigen::Vector3d &p)
    {
        return cv::Point(p(0), p(1));
    }
    static bool verify_loop(const scan::ptr &scan1, const scan::ptr &scan2, const Eigen::Isometry3d &w_tf12)
    {
        return true;
        Eigen::Isometry3d tf_w_to_l = PARAM(T_imu_to_wheel).inverse() * PARAM(T_imu_to_laser);
        Eigen::Isometry3d l_tf_12 = tf_w_to_l.inverse() * w_tf12 * tf_w_to_l;
        int h = PARAM(h_laser_each_scan) / PARAM(laser_resolution) + 1;
        int w = PARAM(w_laser_each_scan) / PARAM(laser_resolution) + 1;
        std::vector<std::vector<int>> map(h, std::vector<int>(w, 0));

        for (int i = 0; i < scan1->points.size(); i++)
        {
            auto [c, r] = xy_to_index(scan1->points[i](0), scan1->points[i](1), w, h);
            if (is_index_valid(r, c, w, h))
                map[r][c] = 1;
        }
        int total = 0;
        int hit = 0;
        for (int i = 0; i < scan2->points.size(); i++)
        {
            auto tmp = scan2->points[i];
            auto tmp2 = l_tf_12 * tmp;
            auto [c, r] = xy_to_index(tmp2(0), tmp2(1), w, h);
            if (is_index_valid(r, c, w, h))
            {
                if (map[r][c] == 1)
                    hit++;
                total++;
            }
        }
        if (total == 0)
            return false;
        double rate = double(hit) / total;
        cv::Mat board(800, 800, CV_8UC3);
        cv::rectangle(board, cv::Point(0, 0), cv::Point(799, 799), cv::Scalar(255, 255, 255), -1);

        Eigen::Vector3d origin(400, 400, 0);
        double scalar = 40;

        for (int i = 0; i < scan1->points.size(); i++)
        {
            Eigen::Vector3d p = origin + scan1->points[i] * scalar;
            if (p(1) > 0 && p(1) < 800 && p(0) > 0 && p(0) < 800)
                board.at<cv::Vec3b>(p(1), p(0)) = {255, 0, 0};
        }
        for (int i = 0; i < scan2->points.size(); i++)
        {
            Eigen::Vector3d p = origin + (l_tf_12 * scan2->points[i]) * scalar;
            if (p(1) > 0 && p(1) < 800 && p(0) > 0 && p(0) < 800)
                board.at<cv::Vec3b>(p(1), p(0)) = {0, 255, 0};
        }
        std::string text = "rate:" + std::to_string(rate);
        cv::putText(board, text,
                    cv::Point(20, 20),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 1, 8);
        cv::imshow("merge", board);
        cv::waitKey(1);
        std::cout << "loop rate:" << rate << std::endl;

        if (rate >= PARAM(verify_loop_rate))
            return true;
        return false;
    }
    static void show_matches_merge(laser_map_feature::ptr m1, laser_map_feature::ptr m2, const Eigen::Isometry3d &w_tf12)
    {
        int w = 400;
        int h = 400;
        double scalar = 20;

        Eigen::Vector3d origin(0, 0, 0);
        Eigen::Vector3d offset(w / 2, h / 2, 0);

        cv::Mat board(h, w, CV_8UC3);

        cv::rectangle(board, cv::Point(0, 0), cv::Point(w - 1, h - 1), cv::Scalar(255, 255, 255), -1);

        double min_x = TIME_MAX, min_y = TIME_MAX, max_x = TIME_MIN, max_y = TIME_MIN;

        Eigen::Isometry3d t_w_w1 = m1->tfs[0] * PARAM(T_imu_to_wheel);
        Eigen::Isometry3d t_w_w2 = m2->tfs[0] * PARAM(T_imu_to_wheel);
        int total_size = 0;
        for (int i = 0; i < m1->scans.size(); i++)
        {
            Eigen::Isometry3d t_w_i = m1->tfs[i] * PARAM(T_imu_to_laser);

            for (int j = 0; j < m1->scans[i]->points.size(); j++)
            {
                Eigen::Vector3d p = t_w_i * m1->scans[i]->points[j];
                if (p(0) < min_x)
                    min_x = p(0);
                if (p(0) > max_x)
                    max_x = p(0);

                if (p(1) < min_y)
                    min_y = p(1);
                if (p(1) > max_y)
                    max_y = p(1);
                total_size++;
                origin += p;
            }
        }

        origin /= -total_size;
        origin(2) = 0;

        // origin(0) = -(min_x + max_x) / 2;
        // origin(1) = -(min_y + max_y) / 2;
        // origin(2) = 0;

        scalar = std::min<double>(w / (max_x - min_x), h / (max_y - min_y));

        for (int i = 0; i < m1->scans.size(); i++)
        {
            Eigen::Isometry3d t_w_i = m1->tfs[i] * PARAM(T_imu_to_laser);
            for (int j = 0; j < m1->scans[i]->points.size(); j++)
            {
                Eigen::Vector3d p = t_w_i * m1->scans[i]->points[j];
                p = (p + origin) * scalar + offset;
                if (p(1) > 0 && p(1) < h && p(0) > 0 && p(0) < w)
                    board.at<cv::Vec3b>(p(1), p(0)) = {255, 0, 0};
            }
        }

        for (int i = 0; i < m2->scans.size(); i++)
        {
            Eigen::Isometry3d t_w2_li = t_w_w2.inverse() * m2->tfs[i] * PARAM(T_imu_to_laser);
            for (int j = 0; j < m2->scans[i]->points.size(); j++)
            {
                Eigen::Vector3d tmp = t_w2_li * m2->scans[i]->points[j];
                Eigen::Vector3d p = t_w_w1 * w_tf12 * tmp;
                p = (p + origin) * scalar + offset;
                if (p(1) > 0 && p(1) < h && p(0) > 0 && p(0) < w)
                {
                    auto &grid = board.at<cv::Vec3b>(p(1), p(0));
                    if (grid == cv::Vec3b{255, 0, 0})
                        grid = cv::Vec3b{255, 255, 0};
                    else
                        board.at<cv::Vec3b>(p(1), p(0)) = {0, 255, 0};
                }
            }
        }

        visualization::get_ptr()->add_image_to_show("merge", board);
        // cv::imshow("merge", board);
        // cv::waitKey(1);
    }
    static void show_matches(laser_map_feature::ptr m1, laser_map_feature::ptr m2, laser_match_point::ptr loop)
    {
        int w = 400;
        int h = 400;
        int line_w = 3;
        double scalar1 = 20;
        double scalar2 = 20;

        Eigen::Vector3d origin1(0, 0, 0);
        Eigen::Vector3d origin2(0, 0.0, 0.0);

        Eigen::Vector3d offset1(w / 2, h / 2, 0);
        Eigen::Vector3d offset2(w / 2 + w + line_w, h / 2, 0);
        cv::Mat board(h, w * 2 + 3, CV_8UC3);

        cv::rectangle(board, cv::Point(0, 0), cv::Point(w * 2 + line_w, h - 1), cv::Scalar(255, 255, 255), -1);
        cv::line(board, cv::Point(w, 0), cv::Point(w, h), cv::Scalar(255, 255, 0), 3);

        double min_x1 = TIME_MAX, min_y1 = TIME_MAX, max_x1 = TIME_MIN, max_y1 = TIME_MIN;

        int total_size1 = 0;
        for (int i = 0; i < m1->scans.size(); i++)
        {
            Eigen::Isometry3d t_w_l = m1->tfs[i] * PARAM(T_imu_to_laser);
            for (int j = 0; j < m1->scans[i]->points.size(); j++)
            {
                Eigen::Vector3d p = t_w_l * m1->scans[i]->points[j];
                if (p(0) < min_x1)
                    min_x1 = p(0);
                if (p(0) > max_x1)
                    max_x1 = p(0);

                if (p(1) < min_y1)
                    min_y1 = p(1);
                if (p(1) > max_y1)
                    max_y1 = p(1);

                origin1 += p;
                total_size1++;
            }
        }
        origin1 /= total_size1;
        origin1 = -origin1;
        origin1(2) = 0;
        // origin1(0) = -(min_x1 + max_x1) / 2;
        // origin1(1) = -(min_y1 + max_y1) / 2;
        // origin1(2) = 0;

        scalar1 = std::min<double>(w / (max_x1 - min_x1), h / (max_y1 - min_y1));

        //-----------------------------------------
        double min_x2 = TIME_MAX, min_y2 = TIME_MAX, max_x2 = TIME_MIN, max_y2 = TIME_MIN;
        int total_size2 = 0;

        for (int i = 0; i < m2->scans.size(); i++)
        {
            Eigen::Isometry3d t_w_l = m2->tfs[i] * PARAM(T_imu_to_laser);
            for (int j = 0; j < m2->scans[i]->points.size(); j++)
            {
                Eigen::Vector3d p = t_w_l * m2->scans[i]->points[j];
                if (p(0) < min_x2)
                    min_x2 = p(0);
                if (p(0) > max_x2)
                    max_x2 = p(0);
                if (p(1) < min_y2)
                    min_y2 = p(1);
                if (p(1) > max_y2)
                    max_y2 = p(1);

                origin2 += p;
                total_size2++;
            }
        }

        origin2 /= total_size2;
        origin2 = -origin2;

        origin2(2) = 0;
        // origin2(0) = -(min_x2 + max_x2) / 2;
        // origin2(1) = -(min_y2 + max_y2) / 2;
        // origin2(2) = 0;

        scalar2 = std::min<double>(w / (max_x2 - min_x2), h / (max_y2 - min_y2));

        double scalar = std::min(scalar1, scalar2);
        for (int i = 0; i < m1->scans.size(); i++)
        {
            Eigen::Isometry3d t_w_l = m1->tfs[i] * PARAM(T_imu_to_laser);
            for (int j = 0; j < m1->scans[i]->points.size(); j++)
            {
                Eigen::Vector3d p = t_w_l * m1->scans[i]->points[j];
                p = (p + origin1) * scalar + offset1;
                if (p(1) > 0 && p(1) < h && p(0) > 0 && p(0) < w)
                    board.at<cv::Vec3b>(p(1), p(0)) = {255, 0, 0};
            }
        }

        for (int i = 0; i < m2->scans.size(); i++)
        {
            Eigen::Isometry3d t_w_l = m2->tfs[i] * PARAM(T_imu_to_laser);
            for (int j = 0; j < m2->scans[i]->points.size(); j++)
            {
                Eigen::Vector3d p = t_w_l * m2->scans[i]->points[j];
                p = (p + origin2) * scalar + offset2;
                if (p(1) > 0 && p(1) < h && p(0) > w + line_w && p(0) < w * 2 + line_w)
                    board.at<cv::Vec3b>(p(1), p(0)) = {0, 255, 0};
            }
        }

        for (int i = 0; i < loop->p1.size(); i++)
        {
            Eigen::Vector3d p1 = (origin1 + loop->p1[i]) * scalar + offset1;
            Eigen::Vector3d p2 = (origin2 + loop->p2[i]) * scalar + offset2;
            cv::line(board, to_cv_Point(p1), to_cv_Point(p2), cv::Scalar(0, 0, 255));
        }
        for (int i = 0; i < m1->points.size(); i++)
        {
            Eigen::Vector3d p1 = (origin1 + m1->points[i]) * scalar + offset1;
            cv::circle(board, to_cv_Point(p1), 3, cv::Scalar(0, 0, 255), 2);
        }
        for (int i = 0; i < m2->points.size(); i++)
        {
            Eigen::Vector3d p2 = (origin2 + m2->points[i]) * scalar + offset2;
            cv::circle(board, to_cv_Point(p2), 3, cv::Scalar(0, 0, 255), 2);
        }

        std::string text = "id:" + std::to_string(m1->index);

        cv::putText(board, text,
                    cv::Point(0, 0) + cv::Point(30, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(255, 0, 0), 1, 8);

        text = "id:" + std::to_string(m2->index);
        cv::putText(board, text,
                    cv::Point(w + line_w, 0) + cv::Point(30, 30),
                    cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 0), 1, 8);

        visualization::get_ptr()->add_image_to_show("matches", board);

        // cv::imshow("matches", board);
        // cv::waitKey(1);
    }
} // namespace lvio_2d

namespace lvio_2d
{

    keyframe_manager::keyframe_manager() : quit(false)
    {
        recorder = new record;
        recorder->set_filename("keyframe");
        // cv::namedWindow("matches");
        // cv::namedWindow("merge");
        // cv::namedWindow("merge");

        cv::waitKey(1);
        laser_frame_count = 0;
        last_loop_index = -1000;
        modify_delta_tf = Eigen::Isometry3d::Identity();
        last_show_time = ros::WallTime::now().toSec();
        last_solve_time = ros::WallTime::now().toSec();
        has_loop_wait_for_solve = false;
        backend_thread = std::thread(std::bind(&keyframe_manager::main_loop, this));
        show_thread = std::thread(std::bind(&keyframe_manager::show_loop, this));
    }

    keyframe_manager::~keyframe_manager()
    {
        delete recorder;
        {
            std::unique_lock<std::mutex> lc(mu);
            quit = true;
            task_cv.notify_one();
        }

        backend_thread.join();
        // output
        if (PARAM(output_tum))
        {
            std::ofstream o_fstream;
            std::string path = PARAM(output_dir) + "back_end.txt";
            o_fstream.open(path, std::ios_base::out);
            o_fstream << "#Time px py pz qx qy qz qw\n";
            o_fstream << std::fixed;
            o_fstream << std::setprecision(10);
            for (int i = 0; i < keyframe_queue.size(); i++)
            {
                Eigen::Isometry3d current_tf_base = lie::make_tf(keyframe_queue[i]->p,
                                                                 keyframe_queue[i]->q) *
                                                    PARAM(T_imu_to_wheel);

                Eigen::Quaterniond q(current_tf_base.matrix().block<3, 3>(0, 0));

                double x = current_tf_base.matrix()(0, 3);
                double y = current_tf_base.matrix()(1, 3);
                double z = current_tf_base.matrix()(2, 3);

                o_fstream << keyframe_queue[i]->time << " " << x << " " << y << " " << z << " " << q.x() << " " << q.y() << " " << q.z() << " " << q.w() << std::endl;
            }
            o_fstream.close();
        }
    }
    void keyframe_manager::add_keyframe(const frame_info::ptr &frame)
    {
        {
            std::unique_lock<std::mutex> lc(mu);
            cache_keyframes.push_back(frame);
        }
        task_cv.notify_one();
    }
    void keyframe_manager::update_other_frame(const std::deque<frame_info::ptr> &frame_infos)
    {
        other_frame = frame_infos;
        {
            std::unique_lock<std::mutex> lc(mu);

            Eigen::Isometry3d current_tf = modify_delta_tf * lie::make_tf(other_frame.back()->p, other_frame.back()->q);
            visualization::get_ptr()->add_status_to_show(current_tf, TRACKING, other_frame.back()->time);
        }
        task_cv.notify_one();
    }
    void keyframe_manager::do_add_keyframe(const frame_info::ptr &frame_ptr)
    {
        recorder->add_record("add keyframe", 1);
        frame_ptr->image = cv::Mat();

        keyframe_queue.push_back(frame_ptr);
        tfs_tracking.push_back(
            lie::make_tf(frame_ptr->p, frame_ptr->q));
        std::tie(keyframe_queue.back()->p, keyframe_queue.back()->q) =
            lie::log_SE3<double>((modify_delta_tf * tfs_tracking.back()));
        if (frame_ptr->type == frame_info::laser)
        {
            laser_frame_count++;
            laser_map_features.push_back(spawn_laser_map_feature());
            recorder->add_record("concers each keyframe", laser_map_features.back()->dess.size());
        }
        else
        {
            laser_map_features.push_back(nullptr);
        }

        if (keyframe_queue.size() > 1)
        {
            int index1 = keyframe_queue.size() - 2;
            int index2 = keyframe_queue.size() - 1;
            std::unique_lock<std::mutex> lc(mu);
            Eigen::Isometry3d tf1 = tfs_tracking[index1];
            Eigen::Isometry3d tf2 = tfs_tracking[index2];
            Eigen::Isometry3d tf12 = tf1.inverse() * tf2;
            seq_edges.emplace_back(new edge(index1, index2, tf12));
        }

        if (frame_ptr->type == frame_info::laser)
        {
            edge::ptr laser_loop = laser_loop_detect();
            if (laser_loop)
            {
                loop_edges.push_back(laser_loop);
                has_loop_wait_for_solve = true;
                last_loop_index = keyframe_queue.size() - 1;
            }
        }
        Eigen::Isometry3d last_frame_tf = tfs_tracking.back();

        if (is_time_to_solve())
        {
            last_solve_time = ros::WallTime::now().toSec();
            recorder->begin_record();
            solve();
            recorder->end_record("solve");
            Eigen::Isometry3d current_frame_tf = lie::make_tf(frame_ptr->p, frame_ptr->q);
            Eigen::Isometry3d delta_tf = current_frame_tf * last_frame_tf.inverse();
            {
                std::unique_lock<std::mutex> lc(mu);
                modify_delta_tf = delta_tf;
            }
            // show_laser_map();
            has_loop_wait_for_solve = false;
        }
        // if (is_time_to_show())
        // {

        //     show_laser_map();
        // }
    }
    void keyframe_manager::show_laser_map()
    {
        last_show_time = ros::WallTime::now().toSec();

        std::vector<Eigen::Isometry3d> path;
        for (int i = 0; i < keyframe_queue.size(); i++)
        {
            path.push_back(lie::make_tf(keyframe_queue[i]->p, keyframe_queue[i]->q));
        }

        visualization::get_ptr()->add_path_to_show("backen_path", path);

        // std::unique_lock<std::mutex> lc(mu);
        if (keyframe_queue.empty())
            return;
        laser_map::ptr laser_map_ptr(new laser_map);

        for (int i = 0; i < keyframe_queue.size(); i++)
        {
            if (keyframe_queue[i]->type != frame_info::laser || !keyframe_queue[i]->laser_match_ptr)
                continue;

            laser_submap::ptr laser_submap_ptr(new laser_submap(keyframe_queue[i]->laser_match_ptr->scan2,
                                                                keyframe_queue[i]->p,
                                                                keyframe_queue[i]->q));
            laser_map_ptr->submaps.push_back(laser_submap_ptr);
        }
        visualization::get_ptr()->add_laser_map_to_show(laser_map_ptr);
    }

    Eigen::Isometry3d ICP_solve_by_opt(std::vector<Eigen::Vector3d> &p1, std::vector<Eigen::Vector3d> &p2,
                                       const Eigen::Isometry3d &init)
    {
        ceres::LossFunction *loss_function = nullptr;
        ceres::LocalParameterization *so3_parameterization =
            factor::so3_parameterization::Create();
        ceres::Problem problem;

        auto [p, q] = lie::log_SE3(init);
        for (int i = 0; i < p1.size(); i++)
        {
            ceres::CostFunction *point_cost_function = point_factor::Create(p1[i], p2[i]);
            problem.AddResidualBlock(
                point_cost_function, loss_function,
                p.data(), q.data());
        }
        problem.SetParameterization(q.data(), so3_parameterization);
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::DENSE_QR;
        options.use_nonmonotonic_steps = true;
        options.minimizer_progress_to_stdout = false;
        ceres::Solver::Summary summary;

        ceres::Solve(options, &problem, &summary);
        return lie::make_tf(p, q);
    }
    bool ICP_RANSAC(std::vector<Eigen::Vector3d> &p1, std::vector<Eigen::Vector3d> &p2,
                    int max_iters, double max_error,
                    Eigen::Isometry3d &ret)
    {
        if (p1.size() < 5)
            return false;
        double min_error = TIME_MAX;
        max_iters = std::min<int>(p1.size(), max_iters);
        std::vector<int> has_use(p1.size(), 0);
        for (int i = 0; i < max_iters; i++)
        {
            // 随机取3个足够不共线的点
            std::vector<Eigen::Vector3d> p1s;
            std::vector<Eigen::Vector3d> p2s;

            // 第一个点随便取
            int p1_index = rand() % p1.size();
            while (has_use[p1_index])
                p1_index = rand() % p1.size();
            has_use[p1_index] = 1;

            p1s.push_back(p1[p1_index]);
            p2s.push_back(p2[p1_index]);

            // 第二个点取距离当前点最远的
            double max_dis = -1;
            int p2_index = 0;
            for (int j = 0; j < p1.size(); j++)
            {
                if (j == p1_index)
                    continue;
                double dis = (p1[j] - p1[p1_index]).norm();
                if (dis > max_dis)
                {
                    max_dis = dis;
                    p2_index = j;
                }
            }
            p1s.push_back(p1[p2_index]);
            p2s.push_back(p2[p2_index]);

            // 第三个点取距离当前线最远的

            max_dis = -1;
            int p3_index = 0;
            for (int j = 0; j < p1.size(); j++)
            {
                if (j == p1_index || j == p2_index)
                    continue;
                double dis = e_laser::dis_from_line(p1[j], p1s[0], p1s[1]);
                if (dis > max_dis)
                {
                    max_dis = dis;
                    p3_index = j;
                }
            }
            p1s.push_back(p1[p3_index]);
            p2s.push_back(p2[p3_index]);

            // 再加两个随便的点
            {
                int tmp_index = rand() % p1.size();
                int p4_index = -1;
                int p5_index = -1;
                while (tmp_index == p1_index || tmp_index == p2_index || tmp_index == p3_index)
                    tmp_index = rand() % p1.size();
                p4_index = tmp_index;

                while (tmp_index == p1_index || tmp_index == p2_index || tmp_index == p3_index || tmp_index == p4_index)
                    tmp_index = rand() % p1.size();
                p5_index = tmp_index;

                p1s.push_back(p1[p4_index]);
                p2s.push_back(p2[p4_index]);

                p1s.push_back(p1[p5_index]);
                p2s.push_back(p2[p5_index]);
            }
            // 求解
            Eigen::Isometry3d tmp = ICP_solve_by_opt(
                p1s, p2s, Eigen::Isometry3d::Identity());

            double error = 0;
            for (int j = 0; j < p1s.size(); j++)
            {
                Eigen::Vector3d diff = p1s[j] - tmp * p2s[j];
                diff(2) = 0;
                error += diff.norm();
            }
            error /= p1s.size();
            if (error < min_error)
            {
                min_error = error;
                ret = tmp;
            }
        }
        if (min_error > max_error)
        {
            return false;
        }

        return true;
    }
    edge::ptr keyframe_manager::laser_loop_detect()
    {
        if (laser_map_features.size() < PARAM(loop_detect_min_interval))
            return nullptr;
        if (!laser_map_features.back())
            return nullptr;
        // if (int(laser_map_features.size()) - last_loop_index < 10)
        // {
        //     return nullptr;
        // }
        // lmicroTimer("laser_loop_detect");
        laser_match_point::ptr laser_loop_match = nullptr;
        for (int i = 0; i < laser_map_features.size() - PARAM(loop_detect_min_interval); i += (PARAM(submap_count) / 3 + 1))
        {
            if (!laser_map_features[i])
                continue;
            laser_loop_match = laser_map_feature::match_map(laser_map_features.back(), laser_map_features[i]);

            if (laser_loop_match)
            {
                laser_loop_match->tf1 = tfs_tracking.back();
                laser_loop_match->tf2 = tfs_tracking[i];
                laser_loop_match->index1 = laser_map_features.size() - 1;
                laser_loop_match->index2 = i;
                laser_loop_match->scan1 = laser_map_features.back()->scans.front();
                laser_loop_match->scan2 = laser_map_features[i]->scans.front();

                //  P1A=P1B=T12 P2B
                std::vector<Eigen::Vector3d> P1As;
                std::vector<Eigen::Vector3d> P2Bs;
                Eigen::Isometry3d tf_inv1 = (laser_loop_match->tf1 * PARAM(T_imu_to_wheel)).inverse();
                Eigen::Isometry3d tf_inv2 = (laser_loop_match->tf2 * PARAM(T_imu_to_wheel)).inverse();
                for (int i = 0; i < laser_loop_match->p1.size(); i++)
                {
                    Eigen::Vector3d p1 = tf_inv1 * laser_loop_match->p1[i];
                    p1(2) = 0;
                    P1As.push_back(p1);
                    Eigen::Vector3d p2 = tf_inv2 * laser_loop_match->p2[i];
                    p2(2) = 0;
                    P2Bs.push_back(p2);
                }
                Eigen::Isometry3d w_T12 = Eigen::Isometry3d::Identity();
                recorder->begin_record();
                w_T12 = ICP_solve_by_opt(P1As, P2Bs, Eigen::Isometry3d::Identity());
                recorder->end_record("ICP_solve_by_opt");

                // recorder->begin_record();
                show_matches_merge(laser_map_features.back(), laser_map_features[i], w_T12);
                // recorder->end_record("show_matches_merge");
                Eigen::Isometry3d i_t12 = PARAM(T_imu_to_wheel) * w_T12 * PARAM(T_imu_to_wheel).inverse();
                Eigen::Isometry3d tracking_i_t12 = laser_loop_match->tf1.inverse() * laser_loop_match->tf2;

                Eigen::Isometry3d error_tf12 = i_t12.inverse() * tracking_i_t12;
                {
                    auto [dp, dq] = lie::log_SE3(error_tf12);
                    if (dp.norm() > PARAM(loop_max_tf_p) || dq.norm() > PARAM(loop_max_tf_q))
                        continue;
                }

                laser_loop_match->tf12 = i_t12;
                edge::ptr ret(new edge(laser_loop_match->index1, laser_loop_match->index2, i_t12));

                if (verify_loop(laser_loop_match->scan1, laser_loop_match->scan2, w_T12))
                {
                    return ret;
                }
            }
        }

        return nullptr;
    }
    void keyframe_manager::get_init()
    {
        for (int i = 0; i < keyframe_queue.size(); i++)
        {
            std::tie(keyframe_queue[i]->p, keyframe_queue[i]->q) = lie::log_SE3(
                tfs_tracking[i]);
        }
    }

    void keyframe_manager::solve()
    {
        int max_loop_times = 2;
        // get_init();
        ceres::LossFunction *loss_function = nullptr;
        ceres::LocalParameterization *so3_parameterization =
            factor::so3_parameterization::Create();
        ceres::Problem problem;

        // seq edge
        for (int i = 0; i < seq_edges.size(); i++)
        {
            int index1 = seq_edges[i]->index1;
            int index2 = seq_edges[i]->index2;
            auto [dp, dq] = lie::log_SE3(seq_edges[i]->tf12);

            // ceres::CostFunction *edge_cost_function = edge_factor::Create(seq_edges[i]->tf12, dp.norm() / PARAM(key_frame_p_motion_threshold));
            ceres::CostFunction *edge_cost_function = edge_factor::Create(seq_edges[i]->tf12, 1);

            problem.AddResidualBlock(
                edge_cost_function, loss_function,
                keyframe_queue[index1]->p.data(), keyframe_queue[index1]->q.data(),
                keyframe_queue[index2]->p.data(), keyframe_queue[index2]->q.data());

            problem.SetParameterization(keyframe_queue[index1]->q.data(), so3_parameterization);
            problem.SetParameterization(keyframe_queue[index2]->q.data(), so3_parameterization);
            if (i == 0)
            {
                problem.SetParameterBlockConstant(keyframe_queue[index1]->q.data());
                problem.SetParameterBlockConstant(keyframe_queue[index1]->p.data());
            }
        }

        // prior factor
        // for (int i = 0; i < keyframe_queue.size(); i++)
        // {

        //     ceres::CostFunction *prior_cost_function = prior_factor::Create(keyframe_queue[i]->p,
        //                                                                     keyframe_queue[i]->q,
        //                                                                     keyframe_queue[i]->sqrt_H);
        //     problem.AddResidualBlock(
        //         prior_cost_function, loss_function,
        //         keyframe_queue[i]->p.data(), keyframe_queue[i]->q.data());

        //     problem.SetParameterization(keyframe_queue[i]->q.data(), so3_parameterization);
        //     if (i == 0)
        //     {
        //         problem.SetParameterBlockConstant(keyframe_queue[i]->p.data());
        //         problem.SetParameterBlockConstant(keyframe_queue[i]->q.data());
        //     }
        // }

        // loop
        for (int i = 0; i < loop_edges.size(); i++)
        {
            int index1 = loop_edges[i]->index1;
            int index2 = loop_edges[i]->index2;

            ceres::CostFunction *edge_cost_function = edge_factor::Create(loop_edges[i]->tf12, PARAM(loop_edge_k));
            problem.AddResidualBlock(
                edge_cost_function, loss_function,
                keyframe_queue[index1]->p.data(), keyframe_queue[index1]->q.data(),
                keyframe_queue[index2]->p.data(), keyframe_queue[index2]->q.data());

            problem.SetParameterization(keyframe_queue[index1]->q.data(), so3_parameterization);
            problem.SetParameterization(keyframe_queue[index2]->q.data(), so3_parameterization);
        }

        if (PARAM(use_ground_p_factor))
        {
            for (int i = 0; i < keyframe_queue.size(); i++)
            {
                ceres::CostFunction *ground_p_cost_function = ground_factor_p::Create();
                problem.AddResidualBlock(
                    ground_p_cost_function, loss_function,
                    keyframe_queue[i]->p.data(),
                    keyframe_queue[i]->q.data());
            }
        }
        if (PARAM(use_ground_q_factor))
        {
            for (int i = 0; i < keyframe_queue.size(); i++)
            {
                ceres::CostFunction *ground_q_cost_function = ground_factor_q::Create();
                problem.AddResidualBlock(
                    ground_q_cost_function, loss_function,
                    keyframe_queue[i]->p.data(),
                    keyframe_queue[i]->q.data());
            }
        }
        ceres::Solver::Options options;
        options.linear_solver_type = ceres::SPARSE_SCHUR;
        options.use_nonmonotonic_steps = false;
        options.minimizer_progress_to_stdout = false;
        // options.max_num_iterations = 10;
        ceres::Solver::Summary summary;

        ceres::Solve(options, &problem, &summary);
        // for (int i = 0; i < loop_edges.size(); i++)
        // {
        //     int index1 = loop_edges[i]->index1;
        //     int index2 = loop_edges[i]->index2; // begin
        //     for (int j = index2 + 1; j <= index1; j++)
        //     {
        //         if (seq_edges[j - 1]->times < max_loop_times)
        //         {
        //             seq_edges[j - 1]->tf12 = lie::make_tf(
        //                                          keyframe_queue[j - 1]->p, keyframe_queue[j - 1]->q)
        //                                          .inverse() *
        //                                      lie::make_tf(
        //                                          keyframe_queue[j]->p, keyframe_queue[j]->q);
        //             seq_edges[j - 1]->times++;
        //         }
        //     }
        // }
        // loop_edges.clear();
    }
    bool keyframe_manager::is_time_to_solve()
    {
        double time_now = ros::WallTime::now().toSec();

        if (has_loop_wait_for_solve && time_now - last_solve_time > 10)
        {
            return true;
        }
        return false;
    }
    bool keyframe_manager::is_time_to_show()
    {
        double time_now = ros::WallTime::now().toSec();
        if (time_now - last_show_time > 5)
        {
            return true;
        }
        return false;
    }

    void keyframe_manager::main_loop()
    {
        while (1)
        {
            frame_info::ptr tmp;
            {
                std::unique_lock<std::mutex> lc(mu);
                while (cache_keyframes.empty())
                {
                    if (quit)
                        return;
                    task_cv.wait(lc);
                }
                tmp = cache_keyframes.front();
                cache_keyframes.pop_front();
            }
            if (quit)
                break;
            do_add_keyframe(tmp);
            if (!cache_keyframes.empty())
                ; // std::cout << "cache_keyframes size:" << cache_keyframes.size() << std::endl;
        }
    }

    void keyframe_manager::show_loop()
    {
        while (1)
        {
            {
                std::unique_lock<std::mutex> lc(mu);
                if (quit)
                    break;
                if (is_time_to_show())
                    show_laser_map();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }

    laser_map_feature::ptr keyframe_manager::spawn_laser_map_feature()
    {
        // std::cout << "spawn_laser_map_feature" << std::endl;
        std::vector<scan::ptr> scans_;
        std::vector<Eigen::Isometry3d> tfs_;
        std::vector<std::vector<Eigen::Vector3d>> concers_;
        int count = 0;
        long long index = -1;
        Eigen::Isometry3d origin = Eigen::Isometry3d::Identity();
        for (int i = keyframe_queue.size() - 1; i > -1; i--)
        {
            if (keyframe_queue[i]->type == frame_info::laser)
            {
                count++;
                scans_.push_back(keyframe_queue[i]->laser_match_ptr->scan2);
                tfs_.push_back(tfs_tracking[i]);
                concers_.push_back(keyframe_queue[i]->laser_concers);
                if (count == PARAM(submap_count))
                    break;
                if (index == -1)
                {
                    index = i;
                    origin = tfs_tracking[i];
                }
            }
        }
        return std::make_shared<laser_map_feature>(concers_, scans_, tfs_, index, origin);
    }

} // namespace lvio_2d

// laser loop
namespace lvio_2d
{

    double f(Eigen::Vector3d v_ij)
    {
        static Eigen::Vector3d unit(1, 0, 0);
        v_ij(2) = 0;
        if (v_ij(1) > 0)
            return std::acos(v_ij.dot(unit) / v_ij.norm());
        return M_PI * 2 - std::acos(v_ij.dot(unit) / v_ij.norm());
    }
    int round(double r)
    {
        return r + 0.5;
    }
    laser_map_feature::laser_map_feature(const std::vector<std::vector<Eigen::Vector3d>> &concers_,
                                         const std::vector<scan::ptr> &scans_,
                                         const std::vector<Eigen::Isometry3d> &tfs_,
                                         const long long &index_,
                                         const Eigen::Isometry3d &origin_)
    {
        // std::cout << "laser_map_feature" << std::endl;
        // collect points
        index = index_;
        origin = origin_;
        for (int i = 0; i < concers_.size(); i++)
        {
            for (int j = 0; j < concers_[i].size(); j++)
            {
                // Remove duplicate points
                bool has_duplicate_points = false;
                for (int k = 0; k < points.size(); k++)
                {
                    Eigen::Vector3d diff = concers_[i][j] - points[k];
                    diff(2) = 0;
                    if (diff.norm() < PARAM(d_res) / 2)
                    {
                        points[k] = (points[k] * 3 + concers_[i][j]) / 4;
                    }
                    if (diff.norm() < PARAM(d_res) * 5)
                    {
                        has_duplicate_points = true;
                        break;
                    }
                }
                if (!has_duplicate_points)
                {
                    points.push_back(concers_[i][j]);
                    // std::cout << points.back().transpose() << std::endl;
                }
            }
        }
        tfs = tfs_;
        scans = scans_;
        std::random_shuffle(points.begin(), points.end());
        int n_points = points.size();

        // 论文算法2
        for (int i = 0; i < n_points; i++)
        {
            des_i tmp_des_i;
            tmp_des_i.i = i;
            for (int j = 0; j < n_points; j++)
            {
                if (j == i)
                    continue;
                Eigen::Vector3d vij = points[j] - points[i];
                vij(2) = 0;
                double aij = f(vij);
                int dij = round(vij.norm() / PARAM(d_res));
                des_j tmp_des_j(aij, dij, j);
                tmp_des_i.des_ij.push_back(tmp_des_j);
            }
            std::sort(
                tmp_des_i.des_ij.begin(),
                tmp_des_i.des_ij.end(),
                [](const des_j &a1, const des_j &a2) -> bool
                {
                    return a1.dij < a2.dij;
                });

            // add quick des
            for (int j = 0; j < tmp_des_i.des_ij.size(); j++)
            {
                tmp_des_i.set_1(tmp_des_i.des_ij[j].dij);
            }
            dess.push_back(tmp_des_i);
        }

        // check
        // std::cout << "n_points:" << n_points << std::endl;
        // for (int i = 0; i < n_points; i++)
        // {
        //     std::cout << "\tdes_i:" << i << std::endl;
        //     for (int j = 0; j < dess[i].des_ij.size(); j++)
        //     {
        //         std::cout << "\t\t[" << dess[i].des_ij[j].dij << " , "
        //                   << dess[i].des_ij[j].aij << " , "
        //                   << dess[i].des_ij[j].j << "]" << std::endl;
        //     }
        // }
    }

    // 论文算法3
    inline laser_match_index::ptr laser_map_feature::match_des(const des_i &d1, const des_i &d2)
    {
        recorder->begin_record();
        recorder->add_record("total match", 1);
        int total_ones = 0;
        for (int i = 0; i < d1.quick_des.size(); i++)
        {
            uint64_t tmp = d1.quick_des[i] & d2.quick_des[i];
            total_ones += CountOnes(tmp);
        }
        if (total_ones < PARAM(laser_loop_min_match_threshold))
        {
            recorder->add_record("filter by quick des", 1);
            recorder->end_record("match_des");
            return nullptr;
        }
        int M = d1.des_ij.size();
        int N = d2.des_ij.size();
        int nAngle = M_PI * 2 / (PARAM(a_res)) + 2;
        int orign = nAngle / 2;
        std::vector<laser_match_index::ptr> match_all(nAngle + 1, nullptr);
        int m = 0;
        int n = 0;
        int maxSize = 0;
        int maxIndex = 0;
        std::vector<int> maybe_true_match_index;
        while (m < M && n < N)
        {
            if (d1.des_ij[m].dij == d2.des_ij[n].dij)
            {
                int tn = 0;
                while (n + tn < N && d1.des_ij[m].dij == d2.des_ij[n + tn].dij)
                {
                    double aDiff_ = d1.des_ij[m].aij - d2.des_ij[n + tn].aij;
                    if (aDiff_ >= M_PI)
                        aDiff_ -= M_PI * 2;
                    else if (aDiff_ < -M_PI)
                        aDiff_ += M_PI * 2;
                    int aDiff = aDiff_ / PARAM(a_res);
                    aDiff += orign;
                    if (!match_all[aDiff])
                    {
                        match_all[aDiff] = laser_match_index::ptr(new laser_match_index);
                        match_all[aDiff]->p1.push_back(d1.i);
                        match_all[aDiff]->p2.push_back(d2.i);
                    }

                    bool has_use = false;
                    for (int i = 0; i < match_all[aDiff]->p1.size(); i++)
                    {
                        if (match_all[aDiff]->p1[i] == d1.des_ij[m].j)
                        {
                            has_use = true;
                            break;
                        }
                    }
                    if (!has_use)
                    {
                        match_all[aDiff]->p1.push_back(d1.des_ij[m].j);
                        match_all[aDiff]->p2.push_back(d2.des_ij[n + tn].j);
                        if (match_all[aDiff]->p1.size() > maxSize)
                        {
                            maxSize = match_all[aDiff]->p1.size();
                            maxIndex = aDiff;
                            maybe_true_match_index = {aDiff};
                        }
                        else if (match_all[aDiff]->p1.size() == maxSize)
                        {
                            maybe_true_match_index.push_back(aDiff);
                        }
                    }
                    tn++;
                }
                m++;
            }
            else if (d1.des_ij[m].dij > d2.des_ij[n].dij)
            {
                n++;
            }
            else
            {
                m++;
            }
        }
        recorder->end_record("match_des");
        if (maybe_true_match_index.empty())
            return match_all[maxIndex];
        else
            return match_all[maybe_true_match_index[rand() % maybe_true_match_index.size()]];
    }

    inline laser_match_point::ptr laser_map_feature::match_map(laser_map_feature::ptr m1, laser_map_feature::ptr m2)
    {
        // RMA
        if (m1->points.size() < PARAM(laser_loop_min_match_threshold) || m2->points.size() < PARAM(laser_loop_min_match_threshold))
            return nullptr;

        auto [dp, dq] = lie::log_SE3<double>(m1->origin.inverse() * m2->origin);
        if (dp.norm() > PARAM(loop_max_dis))
            return nullptr;

        int best_match_size = 0;
        laser_match_index::ptr best_match = nullptr;

        std::vector<int> has_match(m1->points.size(), 0);
        int max_match_times = 5;
        recorder->begin_record();
        for (int j = 0; j < max_match_times; j++)
        {
            int rand_index = rand() % m1->points.size();
            if (has_match[rand_index])
                continue;
            has_match[rand_index] = 1;
            for (int i = 0; i < m2->points.size(); i++)
            {
                laser_match_index::ptr tmp_match = match_des(m1->dess[rand_index], m2->dess[i]);
                if (tmp_match && tmp_match->p1.size() > best_match_size)
                {
                    best_match_size = tmp_match->p1.size();
                    best_match = tmp_match;
                }
            }
        }
        recorder->end_record("match_map");

        laser_match_point::ptr ret = nullptr;

        if (best_match_size > PARAM(laser_loop_min_match_threshold))
        {
            ret = laser_match_point::ptr(new laser_match_point);
            for (int i = 0; i < best_match->p1.size(); i++)
            {
                ret->p1.push_back(m1->points[best_match->p1[i]]);
                ret->p2.push_back(m2->points[best_match->p2[i]]);
            }
            show_matches(m1, m2, ret);
        }
        // check
        // if (best_match_size > PARAM(laser_loop_min_match_threshold))
        // {
        //     std::cout << "find loop" << std::endl;
        //     std::cout << "size:" << best_match_size << std::endl;
        //     for (int i = 0; i < ret->p1.size(); i++)
        //     {
        //         std::cout << ret->p1[i].transpose() << " --> " << ret->p2[i].transpose() << std::endl;
        //     }
        // }

        return ret;
    }

}; // namespace lvio_2d
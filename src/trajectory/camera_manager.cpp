#include "trajectory/camera_manager.h"
#include "utilies/params.h"
#include <algorithm>
// camera manager
namespace lvio_2d
{
    void camera_match::update_scale(const Eigen::Isometry3d &T_1_2)
    {
        assert(pts1.size() == pts2.size());

        Eigen::Matrix3d E = convert::cross_matrix<double>(T_1_2.matrix().block<3, 1>(0, 3)) *
                            T_1_2.matrix().block<3, 3>(0, 0);

        std::vector<Eigen::Vector3d> new_pts1, new_pts2;
        for (int i = 0; i < pts1.size(); i++)
        {
            double error = pts1[i].transpose() * E * pts2[i];
            if (fabs(error) < 1e-4)
            {
                new_pts1.push_back(pts1[i]);
                new_pts2.push_back(pts2[i]);
            }
        }
        pts1 = new_pts1;
        pts2 = new_pts2;
        scales2.resize(pts1.size());
        for (int i = 0; i < pts1.size(); i++)
            scales2[i] = e_cv::triangulate(this->pts1[i], this->pts2[i], T_1_2);
    }

    camera_manager::camera_manager()
    {
        camera_k_inverse = PARAM(camera_K).inverse();
        camera_k = PARAM(camera_K);
        feature_id = 0;
    }
    void camera_manager::abort_id(const long long &id)
    {
        for (int i = 0; i < last_ids.size(); i++)
        {
            if (last_ids[i] == id)
                last_ids[i] = -1;
        }
    }
    camera_match::ptr camera_manager::add_frame(sensor::camera::u_ptr &src)
    {
        //lmicroTimer("add camera frame", myColor::RED, false);

        camera_match::ptr ret(new camera_match);
        if (last_frame.empty())
        {
            last_features.clear();

            std::vector<cv::Point2f> current_features;
            cv::goodFeaturesToTrack(src->image, current_features, PARAM(max_feature_num), 0.01, PARAM(feature_min_dis));
            last_frame = src->image;
            last_features = std::move(current_features);
            for (int i = 0; i < last_features.size(); i++)
                last_ids.push_back(++feature_id);

            ret.reset();
            ret = nullptr;
            return ret;
        }
        if (PARAM(enable_camera_vis))
            ret->m2 = src->color_image;
        std::vector<cv::Point2f> tracking_points;
        std::vector<uchar> status;
        std::vector<float> errors;

        cv::calcOpticalFlowPyrLK(last_frame, src->image, last_features, tracking_points, status, errors);

        std::vector<cv::Point2f> tmp_points;
        std::vector<long long> tmp_id;
        int k = 0;

        for (int i = 0; i < last_features.size(); i++)
        {
            if (status[i] && last_ids[i] != -1)
            {
                Eigen::Vector3d p1_in_pixel(last_features[i].x, last_features[i].y, 1.0);
                Eigen::Vector3d p2_in_pixel(tracking_points[i].x, tracking_points[i].y, 1.0);
                ret->pts1.emplace_back(camera_k_inverse * p1_in_pixel);
                ret->pts2.emplace_back(camera_k_inverse * p2_in_pixel);
                ret->scales2.emplace_back(0);
                ret->id.emplace_back(last_ids[i]);
                k++;
                tmp_points.push_back(tracking_points[i]);
                tmp_id.push_back(last_ids[i]);
            }
        }

        std::vector<cv::Point2f> current_features;
        if (PARAM(max_feature_num) - k > 0)
        {
            cv::goodFeaturesToTrack(src->image, current_features, PARAM(max_feature_num), 0.01, PARAM(feature_min_dis));
            for (int i = 0; i < current_features.size(); i++)
            {
                bool is_too_close = false;
                for (int j = 0; j < tmp_points.size(); j++)
                {
                    auto diff = current_features[i] - tmp_points[j];
                    double r = std::sqrt(diff.x * diff.x + diff.y * diff.y);
                    if (r < PARAM(feature_min_dis))
                        is_too_close = true;
                }
                if (!is_too_close)
                {
                    tmp_points.push_back(current_features[i]);
                    tmp_id.push_back(++feature_id);
                }
            }
        }
        last_frame = src->image;
        last_features = std::move(tmp_points);
        last_ids = std::move(tmp_id);
        return ret;
    }
} // namespace lvio_2d

namespace lvio_2d
{
    feature_info::feature_info(const long long &id_) : id(id_),
                                                       has_estimate(false),
                                                       world_point(0, 0, 0),
                                                       ob_times(0),
                                                       has_ready_for_opt(false),
                                                       linearized_times(0)
    {
    }

    void feature_info::push_frame(const int &frame_index,
                                  const Eigen::Vector3d &camera_point,
                                  const Eigen::Isometry3d &current_tf)
    {
        frame_indexs.push_back(frame_index);
        camera_points.push_back(camera_point);

        if (!has_estimate)
        {
            if (tri_tfs.empty())
            {
                tri_cam_points.push_back(camera_point);
                tri_tfs.push_back(current_tf * PARAM(T_imu_to_camera));
                ob_times++;
            }
            else
            {
                Eigen::Isometry3d tj = current_tf * PARAM(T_imu_to_camera);
                Eigen::Isometry3d ti = tri_tfs.back();
                Eigen::Isometry3d tij = ti.inverse() * tj;
                if (tij.matrix().block<3, 1>(0, 3).norm() > 0.01)
                {
                    tri_cam_points.push_back(camera_point);
                    tri_tfs.push_back(current_tf * PARAM(T_imu_to_camera));
                    ob_times++;
                }
            }
        }
        else
        {
            ob_times++;
        }
    }

    bool feature_info::pop_frame(const int &k)
    {
        int n = 0;
        for (int i = 0; i < frame_indexs.size(); i++)
        {
            frame_indexs[i] -= k;
            if (frame_indexs[i] < 0)
                n++;
        }
        frame_indexs.erase(frame_indexs.begin(), frame_indexs.begin() + n);
        camera_points.erase(camera_points.begin(), camera_points.begin() + n);
        return frame_indexs.empty();
    }
    bool feature_info::estimate_initial_value()
    {
        assert(tri_tfs.size() == tri_cam_points.size());
        if (has_estimate)
            return true;
        if (tri_cam_points.size() < 5)
            return false;

        std::tie(world_point, error) = e_cv::triangulate_points_SVD(tri_cam_points, tri_tfs);
        has_estimate = true;
        return true;
    }
} // namespace lvio_2d

// feature_manger
namespace lvio_2d
{
    feature_manger::feature_manger() : feature_map_ptr(new feature_map)
    {
    }
    void feature_manger::add_match(const camera_match::ptr &match_ptr, const long long &current_index,
                                   const Eigen::Vector3d &current_p,
                                   const Eigen::Vector3d &current_q)
    {
        lastest_frame_features.clear();
        if (!match_ptr)
            return;
        auto current_tf = lie::make_tf(current_p, current_q);
        for (int i = 0; i < match_ptr->id.size(); i++)
        {
            auto [iter, success] = feature_infos.try_emplace(match_ptr->id[i], feature_info::ptr(new feature_info(match_ptr->id[i])));
            iter->second->push_frame(current_index,
                                     match_ptr->pts2[i],
                                     current_tf);
            lastest_frame_features.push_back(iter->second);
        }
    }
    std::vector<long long> feature_manger::pop_frame(const int &k)
    {
        std::vector<long long> remove_id;
        for (auto begin = feature_infos.begin(); begin != feature_infos.end();)
        {
            if (begin->second->pop_frame(k))
            {
                long long id = begin->second->id;
                begin = feature_infos.erase(begin);
                remove_id.push_back(id);
                auto iter = feature_map_ptr->world_points.find(id);
                if (iter != feature_map_ptr->world_points.end())
                    iter->second.has_frozen = true;

                for (int i = 0; i < lastest_frame_features.size(); i++)
                {
                    if (lastest_frame_features[i]->id == id)
                    {
                        std::cout << "erase :" << id << std::endl;
                        lastest_frame_features.erase(lastest_frame_features.begin() + i);
                        break;
                    }
                }
            }
            else
            {
                ++begin;
            }
        }
        return remove_id;
    }

    std::vector<long long> feature_manger::remove_all_features_without_lastest_frame()
    {
        std::unordered_map<long long, bool> remove_id_map;
        for (auto &[id, _] : feature_infos)
            remove_id_map[id] = 1;
        feature_infos.clear();
        for (int i = 0; i < lastest_frame_features.size(); i++)
        {
            feature_infos.emplace(lastest_frame_features[i]->id, lastest_frame_features[i]);
            remove_id_map[lastest_frame_features[i]->id] = 0;
            if (lastest_frame_features[i]->has_ready_for_opt)
            {
                feature_map_ptr->world_points[lastest_frame_features[i]->id] = feature_map_point(lastest_frame_features[i]->world_point, false);
            }
        }
        std::vector<long long> remove_id;
        for (auto &[id, is_remove] : remove_id_map)
        {
            if (is_remove)
            {
                remove_id.push_back(id);

                auto iter = feature_map_ptr->world_points.find(id);
                if (iter != feature_map_ptr->world_points.end())
                    iter->second.has_frozen = true;
            }
        }
        return remove_id;
    }
    std::map<long long, feature_info::ptr> &feature_manger::get_feature_infos()
    {
        return feature_infos;
    }

    std::vector<feature_info::ptr> &feature_manger::get_lastest_frame_features()
    {
        return lastest_frame_features;
    }
    long long feature_manger::remove_feature(const long long &id)
    {
        auto iter = feature_map_ptr->world_points.find(id);
        feature_map_ptr->world_points[id].world_point = feature_infos[id]->world_point;
        feature_map_ptr->world_points[id].has_frozen = true;
        feature_infos.erase(id);
        if (iter != feature_map_ptr->world_points.end())
        {
            iter->second.has_frozen = true;
        }
        for (int i = 0; i < lastest_frame_features.size(); i++)
        {
            if (lastest_frame_features[i]->id == id)
            {
                lastest_frame_features.erase(lastest_frame_features.begin() + i);

                break;
            }
        }
        return id;
    }
    feature_map::ptr &feature_manger::get_feature_map_ptr()
    {
        return feature_map_ptr;
    }

} // namespace lvio_2d
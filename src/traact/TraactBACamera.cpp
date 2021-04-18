/*  BSD 3-Clause License
 *
 *  Copyright (c) 2020, FriederPankratz <frieder.pankratz@gmail.com>
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *     list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *     this list of conditions and the following disclaimer in the documentation
 *     and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its
 *     contributors may be used to endorse or promote products derived from
 *     this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 *  AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 *  IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 *  DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 *  FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 *  DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 *  OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**/

#include "TraactBACamera.h"
#include <traact/util/Logging.h>
#include <traact/math/perspective.h>

traact::Target::Target(bool isStatic, const std::vector<Eigen::Vector3d> &spheres) : isStatic(isStatic),
                                                                                     spheres(spheres) {}

int traact::Target::GetCount() {
    return spheres.size();
}

void traact::TraactBACamera::TrackPoints() {
    tracked_points.clear();
    std::vector<TimestampType> all_timestamps;
    all_timestamps.reserve(measurements.size());
    for(auto const& ts_values: measurements)
        all_timestamps.push_back(ts_values.first);
    std::sort(all_timestamps.begin(), all_timestamps.end());

    TimestampType last_ts = TimestampType::min();
    unsigned int new_track_id = 0;

    for(int i=0;i<all_timestamps.size();++i) {
        TimestampType current_ts = all_timestamps[i];
        //std::vector<Eigen::Vector2d> points = filterPoints(measurements.at(current_ts));
        std::vector<Eigen::Vector2d> points = measurements.at(current_ts);


        std::vector<TrackedPoint2D::Ptr> new_tracked_points;

        if(last_ts == TimestampType::min()) {
            // first init
            for(auto p : points) {
                auto new_tracked_point = std::make_shared<TrackedPoint2D>();
                new_tracked_point->id = new_track_id++;
                new_tracked_point->observations[current_ts] = p;
                new_tracked_points.push_back(new_tracked_point);
            }

        } else {
            //start tracking
            auto& prev_tracked_points = tracked_points[last_ts];
            double max_distance = 32;



            for(auto& tracked_point : prev_tracked_points) {
                if(points.empty())
                    break;
                const auto& p_prev = tracked_point->observations[last_ts];
                double distance = std::numeric_limits<double>().max();
                int closest_point_index = -1;
                int candidate_count = 0;
                for(int point_index = 0;point_index < points.size();++point_index){

                    double d = (points[point_index] - p_prev).norm();
                    if(d < distance && d < max_distance){
                        closest_point_index = point_index;
                        distance = d;
                        candidate_count++;
                    }
                }

                if(candidate_count == 1){
                    tracked_point->observations[current_ts] = points[closest_point_index];
                    points.erase(points.begin()+closest_point_index);
                    new_tracked_points.push_back(tracked_point);
                    //SPDLOG_INFO("point tracked, remaining points: {0}", points.size());
                }
            }
            if(points.size() > 0) {
                //SPDLOG_INFO("new untracked points: {0}", points.size());
                for(auto p : points) {
                    auto new_tracked_point = std::make_shared<TrackedPoint2D>();
                    new_tracked_point->id = new_track_id++;
                    new_tracked_point->observations[current_ts] = p;
                    new_tracked_points.push_back(new_tracked_point);
                }

            } else {
                //SPDLOG_INFO("all points tracked");
            }


        }
        tracked_points[current_ts] = new_tracked_points;

        last_ts = current_ts;
    }

    for(auto tmp : tracked_points){
        for(auto tmp2 : tmp.second) {
            tmp2->Init(max_pixel_error_for_static_point);
        }
    }
    MergeStaticPoints();

}

std::vector<Eigen::Vector2d> traact::TraactBACamera::filterPoints(const std::vector<Eigen::Vector2d> &points) {
    std::vector<Eigen::Vector2d> result;
    for(int i=0;i<points.size();++i) {
        bool good_point = true;
        for(int j=i+1;j<points.size();++j) {
            double distance = (points[i] - points[j]).norm();
            if(distance < 10)
                good_point = false;

        }
        if(good_point)
            result.push_back(points[i]);
    }
    return result;
}

bool traact::TraactBACamera::TryPoseInit() {

    double final_error = std::numeric_limits<double>::max();
    Eigen::Affine3d final_pose;
    std::set<int> prev_candidate_point_ids;

    for(auto& ts_tracks : tracked_points) {
        bool pose_found = false;
        std::vector<TrackedPoint2D::Ptr> candidate_points;
        for(auto& point_track : ts_tracks.second) {
            if(point_track->is_static)
                candidate_points.push_back(point_track);

        }



        if(candidate_points.size() >= root_target->GetCount()){

            /*
            if(!prev_candidate_point_ids.empty()){
                bool same_as_previous = true;
                for(auto tmp : candidate_points) {
                    same_as_previous = same_as_previous && (prev_candidate_point_ids.find(tmp->id) != prev_candidate_point_ids.end());
                }
                if(same_as_previous)
                    continue;
            }*/

            prev_candidate_point_ids.clear();
            for(auto tmp : candidate_points) {
                prev_candidate_point_ids.insert(tmp->id);
            }

            std::vector<size_t> candidate_index;
            Eigen::Affine3d final_candidate_pose;
            double final_candidate_error = std::numeric_limits<double>::max();
            for(int i=0;i<candidate_points.size();++i)
                candidate_index.push_back(i);

            std::sort(candidate_index.begin(), candidate_index.end());
            std::vector<Eigen::Vector2d> image_points;
            image_points.resize(root_target->GetCount());

            do {
                Eigen::Affine3d pose_c2w;
                for(int point_index = 0;point_index < image_points.size();++point_index){
                    //SPDLOG_INFO("model point {0} : candidate track id {1}", point_index, candidate_points[candidate_index[point_index]]->id);
                    image_points[point_index] = candidate_points[candidate_index[point_index]]->observations[ts_tracks.first];
                }

                traact::math::estimate_camera_pose(pose_c2w,image_points,calibration,root_target->spheres);
                double error = traact::math::average_reprojection_error(pose_c2w,image_points, calibration, root_target->spheres);
                if(error < final_candidate_error){
                    //SPDLOG_INFO("new pose candidate with error: {0}", error);
                    final_candidate_error = error;
                    final_candidate_pose = pose_c2w;
                    if(error < 1.0) {
                        SPDLOG_INFO("error smaller then 1 pixel, pose found: {0}", error);
                        pose_found = true;
                        break;
                    }

                }
            } while(std::next_permutation(candidate_index.begin(), candidate_index.end()));

            if(final_candidate_error < final_error) {
                final_pose = final_candidate_pose;
                final_error = final_candidate_error;
            }


        }else if(candidate_points.size() > root_target->GetCount()){
            SPDLOG_WARN("enough points found but currently not supported");
        }

        if(pose_found)
            break;
    }




    if(final_error < std::numeric_limits<double>::max()) {
        SPDLOG_INFO("final world2camera pose with error: {0}", final_error);
        world2camera = final_pose.inverse();
        is_initialized = true;
        return true;
    } else {

    }

    return false;
}

void traact::TraactBACamera::MergeStaticPoints() {

    std::vector<TimestampType> all_timestamps;
    all_timestamps.reserve(tracked_points.size());
    for(auto const& ts_values: tracked_points)
        all_timestamps.push_back(ts_values.first);
    std::sort(all_timestamps.begin(), all_timestamps.end());

    std::map<TimestampType, std::vector<TrackedPoint2D::Ptr> > new_tracked_points;

    TimestampType last_ts = TimestampType::min();
    unsigned int new_track_id = 0;

    for(int i=0;i<all_timestamps.size();++i) {
        TimestampType current_ts = all_timestamps[i];
        std::vector<TrackedPoint2D::Ptr> current_tracked_points;
        for(auto& tracked_point : tracked_points[current_ts]) {
            if(!tracked_point->is_static){
                current_tracked_points.push_back(tracked_point);
                continue;
            }

            bool track_found = false;

            for(auto& tmp : new_tracked_points) {
                for(auto &tmp2 : tmp.second) {
                    if(!tmp2->is_static)
                        continue;
                    if(tracked_point->id == tmp2->id)
                        continue;
                    if((tracked_point->observations[current_ts] - tmp2->observations[tmp.first]).norm() < max_pixel_error_for_static_point) {
                        track_found = true;
                        tmp2->observations.insert(tracked_point->observations.begin(),tracked_point->observations.end());
                        current_tracked_points.push_back(tmp2);
                        break;
                    }
                }
                if(track_found)
                    break;
            }

            if(!track_found)
                current_tracked_points.push_back(tracked_point);
        }

        new_tracked_points.emplace(std::make_pair(current_ts, current_tracked_points));


    }

    tracked_points = new_tracked_points;

}


void traact::TrackedPoint2D::Init(double max_error_movement) {
    if(is_initialized)
        return;
    is_initialized = true;
    if(observations.size() < 15){
        is_static = false;
        return;
    }



    Eigen::Vector2d mean_p(0,0);
    for(auto ts_value : observations){
        mean_p += ts_value.second;
    }
    mean_p = mean_p / observations.size();

    for(auto ts_value : observations){
        if((ts_value.second - mean_p).norm() > max_error_movement){
            is_static = false;
            return;
        }

    }
    is_static = true;
}

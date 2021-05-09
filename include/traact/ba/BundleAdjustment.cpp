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

#include <fstream>
#include "BundleAdjustment.h"

#include <traact/cereal/CerealSpatial.h>
#include <cereal/archives/json.hpp>

#include <traact/math/perspective.h>

#include <traact/ba/cost_function/CeresTargetNPointReprojectionError.h>
#include <traact/ba/cost_function/CeresRefTargetNPointLengthError.h>


void traact::ba::BundleAdjustment::AddCamera(traact::ba::BACamera::Ptr camera) {
    cameras_.push_back(camera);
}

void traact::ba::BundleAdjustment::SetTarget(traact::ba::BATarget::Ptr target) {
    target_ = target;
}

bool traact::ba::BundleAdjustment::CheckData() {
    spdlog::info("Check data consistency");
    used_ts_.clear();

    std::set<TimestampType> all_ts_set;
    for(auto cam : cameras_) {
        auto ts = cam->GetAllTimestamps();
        std::copy(ts.begin(),ts.end(),
                  std::inserter(all_ts_set, all_ts_set.end()));

    }
    std::vector<TimestampType> all_ts = std::vector<TimestampType>(all_ts_set.begin(), all_ts_set.end());
    std::sort(all_ts.begin(), all_ts.end());

    for(TimestampType ts : all_ts) {
        bool valid_ts = true;
        std::size_t cam_count = GetCameraMeaCount(ts);

        // for now at least two cameras must see the points
        // later ease this restriction for reference points (static 3d points)
        if(cam_count < 2)
            continue;
        if(!target_->HasMeasurement(ts)){
            spdlog::info("missing 3d points for target for ts {0}", ts.time_since_epoch().count());
            valid_ts = TryEstimatePoints(ts);
        } else {
            spdlog::info("present 3d points for target for ts {0}", ts.time_since_epoch().count());
        }

        if(valid_ts){
            used_ts_.push_back(ts);
        }
    }


    spdlog::info("Count of valid timestamps for bundle adjustment: {0}", used_ts_.size());

    return true;
}

bool traact::ba::BundleAdjustment::Optimize() {

    ceres_problem_ = std::make_shared<ceres::Problem>();

    target_points_count_ = target_->GetTargetData().size();
    target_parameter_size_ = target_points_count_*3;


    ceres_parameter_ = new double[cameras_.size() * camera_parameter_size_ + used_ts_.size() * target_parameter_size_];
    //ceres_point_parameter_ = new double[used_ts_.size() * size_point_parameter];


    for(int i=0;i<cameras_.size();++i){
        auto cam = cameras_[i];
        double *cam_parameter = GetCameraParameter(i);

        Eigen::Affine3d camera2world = cam->getExtrinsic();

        Eigen::Quaterniond rot(camera2world.rotation());
        Eigen::Vector3d pos(camera2world.translation());


        cam_parameter[0] = rot.w();
        cam_parameter[1] = rot.x();
        cam_parameter[2] = rot.y();
        cam_parameter[3] = rot.z();

        cam_parameter[4] = pos.x();
        cam_parameter[5] = pos.y();
        cam_parameter[6] = pos.z();

    }


    std::vector<std::size_t> camera_mea_count(cameras_.size(), 0);

    for(std::size_t i=0;i<used_ts_.size();++i){
        TimestampType ts = used_ts_[i];

        auto points3d = target_->GetMeasurement(ts);
        double* target_parameter = GetTargetParameter(i);
        for(int j=0;j<points3d.size();++j){
            target_parameter[j * 3 + 0] = points3d[j].x();
            target_parameter[j * 3 + 1] = points3d[j].y();
            target_parameter[j * 3 + 2] = points3d[j].z();
        }

        for(int cam_idx=0;cam_idx<cameras_.size();++cam_idx){
            auto cam = cameras_[cam_idx];
            if(!cam->HasMeasurement(ts))
                continue;
            camera_mea_count[cam_idx]++;
            auto points2d = cam->GetMeasurement(ts);
            auto intrinsic = cam->getIntrinsic();

            std::vector<Eigen::Matrix2d> point_cov;
            point_cov.resize(target_points_count_, Eigen::Matrix2d::Identity());

            ceres::CostFunction* cost_function = CeresTargetNPointReprojectionErrorFactory::Create(points2d, point_cov, intrinsic);

            double *cam_parameter = GetCameraParameter(cam_idx);
            ceres_problem_->AddResidualBlock(cost_function,
                                             NULL,//new ceres::HuberLoss(10), //NULL /* squared loss */,
                                             cam_parameter, target_parameter);

            if(cam->isStaticPosition() || cam->isStaticRotation()) {
                ceres_problem_->SetParameterBlockConstant(cam_parameter);
            }
        }

        if(target_->IsUseTargetResidual()) {

            ceres::CostFunction* cost_function = CeresRefTargetNPointLengthErrorFactory::Create(target_->GetTargetData(), target_->GetStdDev());

            ceres_problem_->AddResidualBlock(cost_function,
                                             new ceres::HuberLoss(10), //NULL /* squared loss */,
                                             target_parameter);
        }
    }



    //ba::PoseLogger pose_logger(directory, &ba_world);
    ceres::Solver::Options options;
    //options.callbacks.push_back(&pose_logger);
    options.update_state_every_iteration = true;

    options.linear_solver_type = ceres::DENSE_SCHUR;
    options.use_inner_iterations = true;
    options.minimizer_progress_to_stdout = true;
    options.logging_type = ceres::PER_MINIMIZER_ITERATION;



    options.gradient_tolerance = 1e-16;
    options.function_tolerance = 1e-12;
    options.parameter_tolerance =1e-8;
    options.max_num_iterations = 50;

    options.num_threads = 8;


    ceres::Solver::Summary summary;

    ceres::Solve(options, ceres_problem_.get(), &summary);
    spdlog::info("{0}", summary.FullReport());

    spdlog::info("Measurements per camera:");
    for(int i=0;i<camera_mea_count.size();++i){
        spdlog::info("camera idx {0}: {1}", i, camera_mea_count[i]);
    }

//    std::ofstream report_stream((experimentdir/"final_solver_report").string());
//    report_stream << summary.FullReport();
//    report_stream.close();


    return summary.IsSolutionUsable();
}

void traact::ba::BundleAdjustment::SaveResult() {
    Eigen::Affine3d pose_tmp = Eigen::Affine3d::Identity();
    pose_tmp.rotate( Eigen::Quaterniond(0,1,0,0));

    Eigen::Affine3d pose_tmp2 = Eigen::Affine3d::Identity();
    pose_tmp2.rotate(Eigen::Quaterniond(0.7071,-0.7071,0,0) * Eigen::Quaterniond(0.7071,0,0,0.7071));

    spatial::Pose6D origin2target = target_to_origin_.inverse();

    for(int i=0;i<cameras_.size();++i){
        auto cam = cameras_[i];
        double* ceres_pose = GetCameraParameter(i);
        Eigen::Quaterniond rot(ceres_pose[0],ceres_pose[1],ceres_pose[2],ceres_pose[3]);
        rot.normalize();
        Eigen::Vector3d pos(ceres_pose[4],ceres_pose[5],ceres_pose[6]);


        Eigen::Affine3d w2c_pose = Eigen::Translation3d(pos) * rot;





        w2c_pose = w2c_pose.inverse();
        Eigen::Affine3d world2camera_unity = pose_tmp2 * w2c_pose * pose_tmp;

        spatial::Pose6D final_world2camera_unity = origin2target * world2camera_unity;

        std::ofstream stream;
        stream.open(cam->getResultfile());

        {
            cereal::JSONOutputArchive archive_(stream);
            archive_(final_world2camera_unity);
        }

        stream.close();
    }

}

std::size_t traact::ba::BundleAdjustment::GetCameraMeaCount(traact::TimestampType ts) {
    std::size_t result = 0;
    for(auto cam : cameras_) {
        if(cam->HasMeasurement(ts))
            result++;
    }
    return result;
}

bool traact::ba::BundleAdjustment::TryEstimatePoints(traact::TimestampType ts) {

    spatial::Position3DList mea_points;
    bool valid = true;
    for(int i=0;i<target_->GetTargetData().size();++i){
        Eigen::Vector3d result;
        valid = valid && TryEstimatePoint(ts, i, result);
        mea_points.push_back(result);
    }

    if(valid)
        target_->SetMeasurement(ts, mea_points);



    return valid;
}

bool traact::ba::BundleAdjustment::TryEstimatePoint(traact::TimestampType ts, std::size_t point_idx,
                                                    Eigen::Vector3d &result) {

    std::vector<Eigen::Affine3d> world2camera;
    std::vector<vision::CameraCalibration> intrinsics;
    std::vector<Eigen::Vector2d> image_point;
    double covariance[16];

    for(auto cam : cameras_) {
        if(!cam->HasMeasurement(ts))
            continue;

        world2camera.push_back(cam->getExtrinsic());
        intrinsics.push_back(cam->getIntrinsic());
        image_point.push_back(cam->GetMeasurement(ts).at(point_idx));
    }

    if(world2camera.size() < 2)
        return false;

    return math::estimate_3d_point(result, world2camera, intrinsics, image_point);

}

traact::ba::BundleAdjustment::~BundleAdjustment() {
    if(ceres_parameter_)
        delete ceres_parameter_;
}

double *traact::ba::BundleAdjustment::GetCameraParameter(std::size_t idx) {
    return ceres_parameter_ + idx * camera_parameter_size_;
}

double *traact::ba::BundleAdjustment::GetTargetParameter(std::size_t idx) {
    double* ceres_point_parameter_ = ceres_parameter_ + cameras_.size() * camera_parameter_size_;
    return ceres_point_parameter_ + idx * target_parameter_size_;
}

void traact::ba::BundleAdjustment::SetTargetToOrigin(traact::spatial::Pose6D pose) {
    target_to_origin_ = pose;

}

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

#include "ba_datatypes.h"
#include <traact/util/Logging.h>
#include <unsupported/Eigen/MatrixFunctions>
#include <traact/math/perspective.h>

#include <boost/numeric/ublas/matrix_expression.hpp>
#include <traact/CeresWandReprojectionError.h>
#include <traact/CeresRefTargetReprojectionError.h>
#include <traact/CeresWandLengthError.h>
#include <traact/CeresRefNPointReprojectionError.h>


traact::vision::CameraCalibration traact::ba::SingleCameraWorld::intrinsic_for_observation(size_t index) {
    return calibration_;
}

void traact::ba::SingleCameraWorld::setCeresProblem(ceres::Problem *problem) {

    wandLength_ = 0.41;//(wand_target_->GetSpheres()[0] - wand_target_->GetSpheres()[1]).norm();

    num_cameras_ = 1;
    parameterSize = 7;
    pointsSize = 6;
    points2DSize = 8;

    //num_points_ = 0;
    num_observations_ = 0;//all_points2d_.size();



    point_index_ = new int[num_observations_];
    camera_index_ = new int[num_observations_];
    observations_ = new double[num_observations_*points2DSize];


    // only camera parameter
    num_parameters_ = parameterSize * num_cameras_;
    parameters_ = new double[num_parameters_];

    //Eigen::Affine3d camera2world = cameras_[cameraIndex]->GetPose_C2W();
    Eigen::Affine3d camera2world = Eigen::Affine3d::Identity();

    Eigen::Quaterniond rot(camera2world.rotation());
    Eigen::Vector3d pos(camera2world.translation());


    parameters_[0] = rot.w();
    parameters_[1] = rot.x();
    parameters_[2] = rot.y();
    parameters_[3] = rot.z();

    parameters_[4] = pos.x();
    parameters_[5] = pos.y();
    parameters_[6] = pos.z();

    const int num_points = 6;

    for(int meaIdx=0;meaIdx<all_points2d_.size();++meaIdx){
        const auto& points2d = all_points2d_[meaIdx];
        const auto& points3d = all_points3d_[meaIdx];
        std::vector<Eigen::Matrix2d> point_cov;

        point_cov.resize(num_points, Eigen::Matrix2d::Identity());

        ceres::CostFunction* cost_function =
                CeresRefNPointReprojectionError<num_points>::Create(points2d, point_cov,
                                                        points3d,
                                                        calibration_);


        problem->AddResidualBlock(cost_function,
                                  NULL,//new ceres::HuberLoss(10), //NULL /* squared loss */,
                                  mutable_camera_for_index(0));
    }

}

void traact::ba::SingleCameraWorld::CommitBA(const std::string& name) {
    WriteResultsNow(name);
}

void traact::ba::SingleCameraWorld::WriteResultsNow(const std::string &filename) {

        double * cam = mutable_camera_for_index(0);

        Eigen::Quaterniond rot(cam[0], cam[1],cam[2],cam[3]);

        rot.normalize();
        Eigen::Vector3d trans(cam[4],cam[5],cam[6]);
        Eigen::Matrix4d pose;
        pose.setIdentity();
        pose.block<3,3>(0,0) = rot.matrix();
        pose.block<3,1>(0,3) = trans;
        Eigen::Affine3d pose_c2w(pose);

        SPDLOG_INFO("raw c2w pos: {0} {1} {2}", trans.x(), trans.y(), trans.z());
        SPDLOG_INFO("raw c2w rot: {0} {1} {2} {3}", rot.x(), rot.y(), rot.z(), rot.w());

        Eigen::Affine3d w2c_pose = pose_c2w.inverse();





        //w2c_pose = w2c_pose * pose_tmp  ;

        std::string final_filename = (experiment_folder_/(filename+std::string(".txt"))).string();
        std::string final_unity_filename = (experiment_folder_/"world2camera.txt").string();


    {
        Eigen::Vector3d result_pos = w2c_pose.translation();
        Eigen::Quaterniond result_rot = Eigen::Quaterniond(w2c_pose.rotation());
        result_rot.normalize();
        std::ostringstream ss;
        ss << "22 serialization::archive 17 0 0 1596216945033476000 0 0 0 0 ";
        ss << result_rot.x() << " ";
        ss << result_rot.y() << " ";
        ss << result_rot.z() << " ";
        ss << result_rot.w() << " 0 0 ";
        ss << result_pos.x() << " ";
        ss << result_pos.y() << " ";
        ss << result_pos.z() << " ";
        std::ofstream ofstream(final_filename);
        ofstream << ss.str();
        ofstream.close();
    }

    Eigen::Affine3d pose_tmp = Eigen::Affine3d::Identity();
    pose_tmp.rotate( Eigen::Quaterniond(0,1,0,0));

        Eigen::Affine3d pose_tmp2 = Eigen::Affine3d::Identity();
        pose_tmp2.rotate(Eigen::Quaterniond(0.7071,-0.7071,0,0) * Eigen::Quaterniond(0.7071,0,0,0.7071));
        //pose_tmp2 = pose_tmp2.inverse();
    Eigen::Affine3d world2camera_unity = pose_tmp2 * w2c_pose * pose_tmp;

    {
        Eigen::Vector3d result_pos = world2camera_unity.translation();
        Eigen::Quaterniond result_rot = Eigen::Quaterniond(world2camera_unity.rotation());
        result_rot.normalize();
        std::ostringstream ss;
        ss << "22 serialization::archive 17 0 0 1596216945033476000 0 0 0 0 ";
        ss << result_rot.x() << " ";
        ss << result_rot.y() << " ";
        ss << result_rot.z() << " ";
        ss << result_rot.w() << " 0 0 ";
        ss << result_pos.x() << " ";
        ss << result_pos.y() << " ";
        ss << result_pos.z() << " ";
        std::ofstream ofstream(final_unity_filename);
        ofstream << ss.str();
        ofstream.close();
    }

}

void traact::ba::SingleCameraWorld::Init(const boost::filesystem::path &experiment_folder) {
    experiment_folder_ = experiment_folder;

}



struct PixelErrorStruct {
    int count{0};
    double error{0};
};

void traact::ba::SingleCameraWorld::reportReprojectionError(){

}

void traact::ba::SingleCameraWorld::addMeasurements(std::vector<Eigen::Vector2d> points2d,
                                                    std::vector<Eigen::Vector3d> points3d) {


    all_points2d_.emplace_back(std::move(points2d));
    all_points3d_.emplace_back(std::move(points3d));
}

void traact::ba::SingleCameraWorld::setCamera(const traact::vision::CameraCalibration &calibration) {
    calibration_ = calibration;
}

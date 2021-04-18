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

#include <utUtil/CalibFile.h>
#include "TraactKinectIRRegistration.h"
#include "Utils.h"
#include <opencv2/highgui.hpp>
#include <traact/vision/UndistortionHelper.h>
#include <traact/math/perspective.h>
#include <traact/util/Logging.h>

#define WITHDEBUG



void traact::TraactKinectIRRegistrationProblem::setCeresProblem(ceres::Problem *problem) {

}

traact::vision::CameraCalibration traact::TraactKinectIRRegistrationProblem::intrinsic_for_observation(size_t index) {
    return vision::CameraCalibration();
}

traact::TraactKinectIRRegistration::TraactKinectIRRegistration(std::string directory) {
    std::vector<Eigen::Vector3d> root_spheres;
    root_spheres.push_back(Eigen::Vector3d(0,0,0));
    root_spheres.push_back(Eigen::Vector3d(0.384,0,0));
    root_spheres.push_back(Eigen::Vector3d(0,0.225,0));
    root_spheres.push_back(Eigen::Vector3d(0.114,0,0));
    Target::Ptr root_target = std::make_shared<Target>(true, root_spheres);

    std::vector<Eigen::Vector3d> wand_spheres;
    wand_spheres.push_back(Eigen::Vector3d(0,0,0));
    wand_spheres.push_back(Eigen::Vector3d(0.41,0,0));
    Target::Ptr wand_target = std::make_shared<Target>(false, wand_spheres);

    std::vector<traact::vision::UndistortionHelper> undistortion;
    std::set<TimestampType > allTimestamps;
    for(int cameraIndex = 0; cameraIndex < 6;cameraIndex++){
        boost::filesystem::path root_dir(directory);
        boost::filesystem::path p = root_dir / (std::string("cn0")+ std::to_string(cameraIndex+1));

        std::cout << "reading path " << p.string() << " index: " << cameraIndex << std::endl;

        m_cameraFolder.push_back(p);






        recording_t localRecording;
        k4a_result_t result = K4A_RESULT_SUCCEEDED;
        std::string k4aFile = (p/"k4a_capture.mkv").string();

        result = k4a_playback_open(k4aFile.c_str(), &localRecording.handle);
        if (result != K4A_RESULT_SUCCEEDED)
            UBITRACK_THROW( "Could not open file \"" + k4aFile  + "\"." );
        result = k4a_playback_get_record_configuration(localRecording.handle, &localRecording.record_config);
        if (result != K4A_RESULT_SUCCEEDED)
        {
            UBITRACK_THROW( "Failed to get record configuration for file: \"" + k4aFile  + "\"." );
        }

        // skip first 5 frames
        for(int i=0;i<5;++i) {
            k4a_stream_result_t stream_result = k4a_playback_get_next_capture(localRecording.handle, &localRecording.capture);
            k4a_capture_release(localRecording.capture);
        }

        m_k4a_recordings.push_back(localRecording);

        k4a_calibration_t k4acalib;

        k4a_result_t functionresult = k4a_playback_get_calibration(localRecording.handle, &k4acalib);

        if(functionresult == K4A_RESULT_FAILED)
            throw std::runtime_error("k4a record error: could not read calibration");

        k4a_calibration_camera_t sensor_calib;
        sensor_calib = k4acalib.depth_camera_calibration;

        vision::CameraCalibration dis_calibration;

        //k4a2traact(sensor_calib, dis_calibration);

        traact::vision::UndistortionHelper undis;
        undis.Init(dis_calibration,true,false,1);
        undistortion.push_back(undis);

        vision::CameraCalibration calibration = undis.GetUndistortedCalibration();


        //std::vector<Ubitrack::Measurement::ErrorPositionList2> points2d = Ubitrack::loadRecord<Ubitrack::Measurement::ErrorPositionList2>((p/"2derrorpoints_irMax6000_distorted.txt").string());
        std::vector<Ubitrack::Measurement::ErrorPositionList2> points2d = Ubitrack::loadRecord<Ubitrack::Measurement::ErrorPositionList2>((p/"2derrorpoints_irMax6000.txt").string());
        std::map<TimestampType ,std::vector<Eigen::Vector2d> > eigen_points2d;
        std::map<TimestampType ,std::vector<Eigen::Matrix2d> > eigen_points2d_covar;

        for(int i=0;i<points2d.size();++i) {
            std::vector<Eigen::Vector2d> current_points;
            std::vector<Eigen::Matrix2d> current_points_covar;
            const auto& current_ubitrack = points2d[i];
            current_points.resize(current_ubitrack->size());
            current_points_covar.resize(current_ubitrack->size());
            TimestampType ts = TimestampType::min() + std::chrono::nanoseconds(current_ubitrack.time());
            for(int j=0;j<current_ubitrack->size();++j) {
                current_points[j].x() = current_ubitrack->at(j).value[0];
                current_points[j].y() = current_ubitrack->at(j).value[1];

                for(int row_index = 0 ; row_index < 2; row_index++) {
                    for (int col_index = 0; col_index < 2; col_index++) {
                        current_points_covar[j](row_index,col_index) = current_ubitrack->at(j).covariance.at_element(row_index, col_index);
                    }

                }


            }
            //std::vector<Eigen::Vector2d> undis_current_points;
            //math::undistort_points(dis_calibration, calibration, dis_current_points, undis_current_points);
            eigen_points2d[ts] = current_points;
            eigen_points2d_covar[ts] = current_points_covar;

            allTimestamps.insert(ts);
        }
        m_points2d.push_back(eigen_points2d);
        m_points2d_covar.push_back(eigen_points2d_covar);


        auto newCamera = std::make_shared<TraactBACamera>();
        newCamera->calibration = calibration;
        newCamera->measurements = eigen_points2d;
        newCamera->wand_target = wand_target;
        newCamera->root_target = root_target;
        newCamera->id = cameraIndex;

        cameras_.push_back(newCamera);

    }

    for(auto& camera : cameras_) {
        camera->TrackPoints();
    }

    m_allTimestamps.insert(m_allTimestamps.end(), allTimestamps.begin(),allTimestamps.end());
    std::sort(m_allTimestamps.begin(),m_allTimestamps.end());

    for(auto& camera : cameras_) {
        if(camera->TryPoseInit()) {
            SPDLOG_INFO("found initial pose from root target for camera {0}", camera->id);
            std::stringstream ss;
            ss << "\n" << camera->world2camera.matrix();
            SPDLOG_INFO(ss.str());
        } else {
            SPDLOG_INFO("could not find initial pose from root target for camera {0}", camera->id);
        }
    }

    writeResultFiles();

#ifdef WITHDEBUG
    char key = ' ';



    for(int ts_index = 0; ts_index < m_allTimestamps.size() && key != 'e';++ ts_index) {

        TimestampType  current_ts = m_allTimestamps[ts_index];

        for (int cameraIndex = 0; cameraIndex < m_k4a_recordings.size(); ++cameraIndex) {
            std::string windowName = "Camera " + std::to_string(cameraIndex);
            cv::namedWindow(windowName, CV_WINDOW_FREERATIO);
        }


        for (int cameraIndex = 0; cameraIndex < m_k4a_recordings.size(); ++cameraIndex) {
            auto &recording = m_k4a_recordings[cameraIndex];

            k4a_stream_result_t stream_result = k4a_playback_get_next_capture(recording.handle, &recording.capture);

            k4a_image_t image = k4a_capture_get_ir_image(recording.capture);
            cv::Mat current_image;
            if(image == NULL){
                current_image = cv::Mat(576,640, CV_8UC4);
                SPDLOG_ERROR("no image for camera : {0}", cameraIndex);

            }  else {

                cv::Mat img;
                u_int8_t* bufferp = k4a_image_get_buffer(image);
                size_t bufferSize = k4a_image_get_size(image);
                int stride = k4a_image_get_stride_bytes(image);
                int w = k4a_image_get_width_pixels(image );
                int h = k4a_image_get_height_pixels(image );
                img = cv::Mat(cv::Size(w, h), CV_16UC1, static_cast<void *>(bufferp), static_cast<size_t>(stride)).clone();

                cv::Mat tmp, tmp_color;
                img.convertTo(tmp, CV_MAKETYPE(CV_MAT_DEPTH(CV_8UC1), 1), 255. / 1000.0);



                cv::cvtColor(tmp, tmp_color, cv::COLOR_GRAY2BGR);
                undistortion[cameraIndex].UndistortImage(tmp_color, current_image);

                k4a_image_release(image);
                k4a_capture_release(recording.capture);
            }

            auto& current_cam = cameras_[cameraIndex];

            {
                auto find_result = current_cam->measurements.find(current_ts);
                if (find_result != current_cam->measurements.end()) {
                    auto &measured_points = find_result->second;
                    for (auto measured_point : measured_points) {
                        cv::circle(current_image, cv::Point2d(measured_point.x(), measured_point.y()), 2, cv::Scalar(0, 255, 0));
                    }
                }
                if(current_cam->is_initialized){
                    for(int i=0; i< root_target->GetCount();++i){
                        auto model_point = root_target->spheres[i];
                        auto reproj_point = traact::math::reproject_point(current_cam->world2camera.inverse(), current_cam->calibration, model_point);
                        cv::circle(current_image, cv::Point2d(reproj_point.x(), reproj_point.y()), 6, cv::Scalar(0, 255, 255));

                        cv::putText(current_image, std::to_string(i), cv::Point2d(reproj_point.x() + 2, reproj_point.y() - 15),
                                    cv::HersheyFonts::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, cv::Scalar(0, 255, 255));
                    }
                }
            }

            {
                auto find_result = current_cam->tracked_points.find(current_ts);
                if (find_result != current_cam->tracked_points.end()) {
                    auto &tracked_points = find_result->second;
                    for (auto tracked_point : tracked_points) {
                        unsigned int id = tracked_point->id;
                        Eigen::Vector2d point = tracked_point->observations[current_ts];
                        bool isStaticPoint = tracked_point->is_static;
                        cv::Scalar textColor;
                        if(isStaticPoint) {
                            textColor = cv::Scalar(0,0,255);
                        } else {
                            textColor = cv::Scalar(255,0,0);
                        }
                        cv::circle(current_image, cv::Point2d(point.x(), point.y()), 4, cv::Scalar(255, 0, 0));
                        cv::putText(current_image, std::to_string(id), cv::Point2d(point.x() + 2, point.y() - 5),
                                    cv::HersheyFonts::FONT_HERSHEY_SCRIPT_SIMPLEX, 0.5, textColor);



                    }
                }
            }


            std::string windowName = "Camera " + std::to_string(cameraIndex);
            cv::imshow(windowName, current_image);

        }
        if(key == ' ') {
            key = cv::waitKey(0);
        } else {
            key = cv::waitKey(1);
        }
    }
#endif




}

traact::TraactProblemBase *traact::TraactKinectIRRegistration::getProblem() {
    return nullptr;
}

void traact::TraactKinectIRRegistration::writeResultFiles() {
    for(size_t cameraIndex =0; cameraIndex < m_cameraFolder.size();++cameraIndex) {
        //double * cam = m_problem.mutable_cameras() + m_problem.parameterSize*cameraIndex;

        //Ubitrack::Math::Quaternion rot(cam[1],cam[2],cam[3],cam[0]);
        Eigen::Quaterniond qrot(cameras_[cameraIndex]->world2camera.rotation());

        Ubitrack::Math::Quaternion rot(qrot.x(), qrot.y(), qrot.z(), qrot.w());
        rot = rot.normalize();
        //Ubitrack::Math::Vector3d trans(cam[4],cam[5],cam[6]);
        Eigen::Vector3d tpos = cameras_[cameraIndex]->world2camera.translation();
        Ubitrack::Math::Vector3d trans(tpos.x(),tpos.y(),tpos.z());
        Ubitrack::Math::Pose pose(rot,trans);

        Ubitrack::Math::Pose pose_tmp(Ubitrack::Math::Quaternion(1,0,0,0),Ubitrack::Math::Vector3d(0,0,0));

        //Ubitrack::Math::Pose world2cam = ~pose;
        Ubitrack::Math::Pose world2cam = pose;

        {
            std::string file = (m_cameraFolder[cameraIndex]/"world2camera_ba.txt").string();

            Ubitrack::Measurement::Pose result(Ubitrack::Measurement::now(), world2cam);

            Ubitrack::Util::writeCalibFile(file, result );
        }
        {
            std::string file = (m_cameraFolder[cameraIndex]/"world2camera_ba_inv.txt").string();

            Ubitrack::Measurement::Pose result(Ubitrack::Measurement::now(), world2cam * pose_tmp);

            Ubitrack::Util::writeCalibFile(file, result );
        }



    }
}

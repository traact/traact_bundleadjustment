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

#ifndef TRAACTMULTI_TRAACTKINECTIRREGISTRATION_H
#define TRAACTMULTI_TRAACTKINECTIRREGISTRATION_H

#include <utMeasurement/Measurement.h>
#include <map>
#include <set>
#include <vector>
#include <utUtil/Exception.h>
#include <boost/filesystem/path.hpp>

#include <malloc.h>
#include <k4a/k4a.h>
#include <k4arecord/k4arecord_export.h>
#include <k4arecord/playback.h>
#include <k4arecord/record.h>
#include <utVision/Undistortion.h>

#include "UbitrackProblem.h"
#include "TraactBACamera.h"
#include <traact/vision_datatypes.h>
#include <Eigen/Core>

namespace traact {

    typedef struct
    {
        k4a_playback_t handle;
        k4a_record_configuration_t record_config;
        k4a_capture_t capture;
    } recording_t;

    class TraactProblemBase {

    public:
        virtual ~TraactProblemBase() {
            delete[] point_index_;
            delete[] camera_index_;
            delete[] observations_;
            delete[] parameters_;
        }

        int num_observations() const { return num_observations_; }

        const double *observations() const { return observations_; }

        double *mutable_cameras() { return parameters_; }

        double *mutable_points() { return parameters_ + parameterSize * num_cameras_; }

        double *mutable_camera_for_observation(int i) {
            return mutable_cameras() + camera_index_[i] * parameterSize;
        }

        double *mutable_point_for_observation(int i) {
            return mutable_points() + point_index_[i] * pointsSize;
        }

        double *measurement_for_observation(int i) {
            return observations_ + points2DSize * i;
        }

        virtual vision::CameraCalibration intrinsic_for_observation(size_t index) = 0;

        virtual void setCeresProblem(ceres::Problem* problem) = 0;


        //protected:

        int num_cameras_;
        int num_points_;
        int num_observations_;
        int num_parameters_;

        int *point_index_;
        int *camera_index_;
        double *observations_;
        double *parameters_;

        size_t parameterSize=7;
        size_t pointsSize=3;
        size_t points2DSize=2;
    };




    class TraactKinectIRRegistrationProblem : public TraactProblemBase {
    public:
        TraactKinectIRRegistrationProblem()= default;

        vision::CameraCalibration intrinsic_for_observation(size_t index) override;

        void setCeresProblem(ceres::Problem *problem) override;


    };

    class TraactKinectIRRegistration {
    public:
        TraactKinectIRRegistration(std::string directory);

        TraactProblemBase *getProblem();

        void writeResultFiles();

    protected:

        // input from circle detection
        std::vector<std::map<TimestampType ,std::vector<Eigen::Vector2d> > > m_points2d;
        std::vector<std::map<TimestampType ,std::vector<Eigen::Matrix2d> > > m_points2d_covar;

        // input from video files
        std::vector<vision::CameraCalibration> m_calibration;

        std::vector<TimestampType > m_allTimestamps;

        std::vector<boost::filesystem::path> m_cameraFolder;
        std::vector<recording_t> m_k4a_recordings;

        TraactKinectIRRegistrationProblem m_problem;

        std::vector<TraactBACamera::Ptr> cameras_;


    };
}




#endif //TRAACTMULTI_TRAACTKINECTIRREGISTRATION_H

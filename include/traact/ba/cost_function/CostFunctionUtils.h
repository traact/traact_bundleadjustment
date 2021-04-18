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

#ifndef TRAACTMULTI_COSTFUNCTIONUTILS_H
#define TRAACTMULTI_COSTFUNCTIONUTILS_H

#include <traact/vision_datatypes.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <cmath>
#include <sstream>

namespace traact::ba {
    template<typename T> inline void reprojectPoint(const T* const camera,const T* const point, const double fx, const double fy, const double cx, const double cy, T* reprojectedPoint) {

        // camera[0,1,2,3] is are the rotation of the camera as a quaternion.
        //
        // We use QuaternionRotatePoint as it does not assume that the
        // quaternion is normalized, since one of the ways to run the
        // bundle adjuster is to let Ceres optimize all 4 quaternion
        // parameters without a local parameterization.
        T p[3];
        ceres::QuaternionRotatePoint(camera, point, p);

        p[0] += camera[4];
        p[1] += camera[5];
        p[2] += camera[6];


        const T xp = fx * p[0] + cx * p[2];
        const T yp = fy * p[1] + cy * p[2];
        const T zp = p[2];

        reprojectedPoint[0] = xp / zp;
        reprojectedPoint[1] = yp / zp;

    };


    struct CeresBAProblemBase {

        virtual ~CeresBAProblemBase() {
            if(point_index_)
                delete[] point_index_;
            if(camera_index_)
                delete[] camera_index_;
            if(observations_)
                delete[] observations_;
            if(parameters_)
                delete[] parameters_;
        }

        int num_observations() const { return num_observations_; }

        const double *observations() const { return observations_; }

        double *mutable_cameras() { return parameters_; }

        double *mutable_points() { return parameters_ + parameterSize * num_cameras_; }

        double* mutable_wand_length(){
            return parameters_+num_parameters_-1;
        }

        double *mutable_camera_for_observation(int i) {
            return mutable_cameras() + camera_index_[i] * parameterSize;
        }

        double *mutable_camera_for_index(int i) {
            return mutable_cameras() + i * parameterSize;
        }

        double *mutable_point_for_observation(int i) {
            return mutable_points() + point_index_[i] * pointsSize;
        }

        double *measurement_for_observation(int i) {
            return observations_ + points2DSize * i;
        }

        Eigen::Matrix2d covariance_for_observation(int observation_index, int point_index) {
            return observations_covar_[observation_index][point_index];
        }


        virtual traact::vision::CameraCalibration intrinsic_for_observation(size_t index) = 0;

        virtual void setCeresProblem(ceres::Problem* problem) = 0;


        //protected:

        int num_cameras_;
        int num_points_;
        int num_observations_;
        int num_parameters_;

        int *point_index_{0};
        int *camera_index_{0};
        double *observations_{0};
        std::vector<std::vector<Eigen::Matrix2d> > observations_covar_;
        double *parameters_{0};
        double wandLength_{0};

        size_t parameterSize=7;
        size_t pointsSize=3;
        size_t points2DSize=2;


    };
}

#endif //TRAACTMULTI_COSTFUNCTIONUTILS_H

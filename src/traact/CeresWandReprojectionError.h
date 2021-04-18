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

#ifndef TRAACTMULTI_CERESWANDREPROJECTIONERROR_H
#define TRAACTMULTI_CERESWANDREPROJECTIONERROR_H

#include <traact/vision_datatypes.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <cmath>
#include <sstream>
namespace traact::ba{
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

    struct CeresWandReprojectionError {
        CeresWandReprojectionError(const double* observed_1_, const Eigen::Matrix2d observed_1_cov_, const double* observed_2_, const Eigen::Matrix2d observed_2_cov_, traact::vision::CameraCalibration intrinsics)
                : observed_1(observed_1_), observed_2(observed_2_)
                , observed_1_cov(observed_1_cov_), observed_2_cov(observed_2_cov_)
                , m_intrinsics(intrinsics)
                , fx(intrinsics.fx), fy(intrinsics.fy)
                , cx(intrinsics.cx), cy(intrinsics.cy)
        {
            stddev_1[0] = sqrt(observed_1_cov(0,0));
            stddev_1[1] = sqrt(observed_1_cov(1,1));

            stddev_2[0] = sqrt(observed_2_cov(0,0));
            stddev_2[1] = sqrt(observed_2_cov(1,1));

        }

        template <typename T>
        bool operator()(const T* const camera,
                        const T* const point,
                        T* residuals) const {



            T p1[2];
            T p2[2];

            reprojectPoint(camera, point, fx,fy,cx,cy,p1);

            reprojectPoint(camera, point+3, fx,fy,cx,cy,p2);


            // The error is the difference between the predicted and observed position.
            residuals[0] = (observed_1[0] - p1[0]) / T(stddev_1[0]);
            residuals[1] = (observed_1[1] - p1[1]) / T(stddev_1[1]);

            residuals[2] = (observed_2[0] - p2[0]) / T(stddev_2[0]);
            residuals[3] = (observed_2[1] - p2[1]) / T(stddev_2[1]);

//            {
//                std::stringstream  ss;
//                ss << "UbitrackWandReprojectionError "  << "\n";
//                ss << "observed_x " << observed_1[0] << " predicted_x " << p1[0]  << "\n";
//                ss<< "observed_y " << observed_1[1] << " predicted_y " << p1[1] << std::endl;
//                ss << "residuals p1 " << residuals[0] << " " << residuals[1] << std::endl;
//                ss << "residuals p2 " << residuals[2] << " " << residuals[3] << std::endl;
//
//                ss <<  "\n";
//                spdlog::info(ss.str());
//            }

            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const double* observed_1,
                                           const Eigen::Matrix2d observed_1_cov,
                                           const double* observed_2,
                                           const Eigen::Matrix2d observed_2_cov,
                                           const traact::vision::CameraCalibration intrinsics) {


            return (new ceres::AutoDiffCostFunction< CeresWandReprojectionError, 4, 7, 6 >(
                    new CeresWandReprojectionError(observed_1,observed_1_cov, observed_2 ,observed_2_cov, intrinsics)));
        }

        const Eigen::Vector2d observed_1;
        const Eigen::Vector2d observed_2;
        const Eigen::Matrix2d observed_1_cov;
        const Eigen::Matrix2d observed_2_cov;

        double stddev_1[2];
        double stddev_2[2];

        const double fx;
        const double fy;

        const double cx;
        const double cy;

        traact::vision::CameraCalibration m_intrinsics;
    };
}

#endif //TRAACTMULTI_CERESWANDREPROJECTIONERROR_H

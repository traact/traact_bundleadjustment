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

#ifndef TRAACTMULTI_CERESREFTARGETREPROJECTIONERROR_H
#define TRAACTMULTI_CERESREFTARGETREPROJECTIONERROR_H

#include "CeresWandReprojectionError.h"

namespace traact::ba {
    struct CeresRefTargetReprojectionError {
        CeresRefTargetReprojectionError(const Eigen::Vector2d observed_1,
                                        const Eigen::Matrix2d observed_1_cov,
                                        const Eigen::Vector2d observed_2,
                                        const Eigen::Matrix2d observed_2_cov,
                                        const Eigen::Vector2d observed_3,
                                        const Eigen::Matrix2d observed_3_cov,
                                        const Eigen::Vector2d observed_4,
                                        const Eigen::Matrix2d observed_4_cov,
                                        const Eigen::Vector3d &p1,
                                        const Eigen::Vector3d &p2, const Eigen::Vector3d &p3, const Eigen::Vector3d &p4,
                                        const vision::CameraCalibration &mIntrinsics) : observed_1_(observed_1),
                                                                                        observed_2_(observed_2),
                                                                                        observed_3_(observed_3),
                                                                                        observed_4_(observed_4),
                                                                                        observed_1_cov_(observed_1_cov),
                                                                                        observed_2_cov_(observed_2_cov),
                                                                                        observed_3_cov_(observed_3_cov),
                                                                                        observed_4_cov_(observed_4_cov), p1(p1),
                                                                                        p2(p2), p3(p3), p4(p4),
                                                                                        m_intrinsics(mIntrinsics)
                , fx(mIntrinsics.fx), fy(mIntrinsics.fy)
                , cx(mIntrinsics.cx), cy(mIntrinsics.cy){

            stddev_1_[0] = sqrt(observed_1_cov_(0,0));
            stddev_1_[1] = sqrt(observed_1_cov_(1,1));

            stddev_2_[0] = sqrt(observed_2_cov_(0,0));
            stddev_2_[1] = sqrt(observed_2_cov_(1,1));

            stddev_3_[0] = sqrt(observed_3_cov_(0,0));
            stddev_3_[1] = sqrt(observed_3_cov_(1,1));

            stddev_4_[0] = sqrt(observed_4_cov_(0,0));
            stddev_4_[1] = sqrt(observed_4_cov_(1,1));
        }

        template <typename T>
        bool operator()(const T* const camera,
                        T* residuals) const {



            T p3d1[3];
            T p3d2[3];
            T p3d3[3];
            T p3d4[3];

            for(int i=0;i<3;i++){
                p3d1[i] = T(p1[i]);
                p3d2[i] = T(p2[i]);
                p3d3[i] = T(p3[i]);
                p3d4[i] = T(p4[i]);
            }

            T p1[2];
            T p2[2];
            T p3[2];
            T p4[2];

            reprojectPoint(camera, p3d1, fx,fy,cx,cy,p1);
            reprojectPoint(camera, p3d2, fx,fy,cx,cy,p2);
            reprojectPoint(camera, p3d3, fx,fy,cx,cy,p3);
            reprojectPoint(camera, p3d4, fx,fy,cx,cy,p4);


            // The error is the difference between the predicted and observed position.
            residuals[0] = (observed_1_[0] - p1[0]) / T(stddev_1_[0]);
            residuals[1] = (observed_1_[1] - p1[1]) / T(stddev_1_[1]);

            residuals[2] = (observed_2_[0] - p2[0]) / T(stddev_2_[0]);
            residuals[3] = (observed_2_[1] - p2[1]) / T(stddev_2_[1]);

            residuals[4] = (observed_3_[0] - p3[0]) / T(stddev_3_[0]);
            residuals[5] = (observed_3_[1] - p3[1]) / T(stddev_3_[1]);

            residuals[6] = (observed_4_[0] - p4[0]) / T(stddev_4_[0]);
            residuals[7] = (observed_4_[1] - p4[1]) / T(stddev_4_[1]);


//            {
//                std::stringstream  ss;
//                ss << "UbitrackWandReprojectionError "  << "\n";
//                //ss << "observed_x " << observed_1_[0] << " predicted_x " << p1[0]  << "\n";
//                //ss<< "observed_y " << observed_1_[1] << " predicted_y " << p1[1] << std::endl;
//                ss << "residuals p1 " << residuals[0] << " " << residuals[1] << std::endl;
//                ss << "residuals p2 " << residuals[2] << " " << residuals[3] << std::endl;
//                ss << "residuals p3 " << residuals[4] << " " << residuals[5] << std::endl;
//                ss << "residuals p4 " << residuals[6] << " " << residuals[7] << std::endl;
//
//                ss <<  "\n";
//                spdlog::info(ss.str());
//            }



            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const Eigen::Vector2d observed_1,
                                           const Eigen::Matrix2d observed_1_cov,
                                           const Eigen::Vector2d observed_2,
                                           const Eigen::Matrix2d observed_2_cov,
                                           const Eigen::Vector2d observed_3,
                                           const Eigen::Matrix2d observed_3_cov,
                                           const Eigen::Vector2d observed_4,
                                           const Eigen::Matrix2d observed_4_cov,
                                           const Eigen::Vector3d p1,
                                            const Eigen::Vector3d p2,
                                            const Eigen::Vector3d p3,
                                            const Eigen::Vector3d p4,
                                           const traact::vision::CameraCalibration intrinsics) {


            return (new ceres::AutoDiffCostFunction< CeresRefTargetReprojectionError, 8, 7 >(
                    new CeresRefTargetReprojectionError(observed_1, observed_1_cov,
                                                        observed_2, observed_2_cov,
                                                        observed_3, observed_3_cov,
                                                        observed_4, observed_4_cov, p1,p2,p3,p4,intrinsics)));
        }

        const Eigen::Vector2d observed_1_;
        const Eigen::Vector2d observed_2_;
        const Eigen::Vector2d observed_3_;
        const Eigen::Vector2d observed_4_;
        const Eigen::Matrix2d observed_1_cov_;
        const Eigen::Matrix2d observed_2_cov_;
        const Eigen::Matrix2d observed_3_cov_;
        const Eigen::Matrix2d observed_4_cov_;

        double stddev_1_[2];
        double stddev_2_[2];
        double stddev_3_[2];
        double stddev_4_[2];

        const Eigen::Vector3d p1;
        const Eigen::Vector3d p2;
        const Eigen::Vector3d p3;
        const Eigen::Vector3d p4;

        const double fx;
        const double fy;

        const double cx;
        const double cy;

        traact::vision::CameraCalibration m_intrinsics;
    };
}


#endif //TRAACTMULTI_CERESREFTARGETREPROJECTIONERROR_H

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

#ifndef TRAACTMULTI_CERESREFNPOINTEPREROJECTIONERROR_H
#define TRAACTMULTI_CERESREFNPOINTEPREROJECTIONERROR_H

#include "CostFunctionUtils.h"

namespace traact::ba {


    template<uint16_t N>
    struct CeresRefNPointReprojectionError {
        CeresRefNPointReprojectionError(const std::vector<Eigen::Vector2d> observed,
                                        const std::vector<Eigen::Matrix2d> observed_cov,
                                        const std::vector<Eigen::Vector3d> ref_point,
                                        const vision::CameraCalibration &mIntrinsics) : observed_(observed),
                                                                                        observed_cov_(observed_cov),
                                                                                        ref_points_(ref_point),
                                                                                        m_intrinsics(mIntrinsics)
                , fx(mIntrinsics.fx), fy(mIntrinsics.fy)
                , cx(mIntrinsics.cx), cy(mIntrinsics.cy){

            stddev_.resize(N, std::vector<double>(2,1));

            for(int i=0;i<N;++i){
                stddev_[i][0] = sqrt(observed_cov_[i](0,0));
                stddev_[i][1] = sqrt(observed_cov_[i](1,1));
            }

        }

        template <typename T>
        bool operator()(const T* const camera,
                        T* residuals) const {



            T p3d[N][3];


            for(int i=0;i<N;i++){
                for(int j=0;j<3;j++){
                    p3d[i][j] = T(ref_points_[i][j]);
                }
            }


            T p[N][2];
            for(int i=0;i<N;i++){
                reprojectPoint(camera, p3d[i], fx,fy,cx,cy,p[i]);
            }

            for(int i=0;i<N;i++){
                // The error is the difference between the predicted and observed position.
                residuals[i*2+0] = (observed_[i][0] - p[i][0]) / T(stddev_[i][0]);
                residuals[i*2+1] = (observed_[i][1] - p[i][1]) / T(stddev_[i][1]);
            }


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
        static ceres::CostFunction* Create(const std::vector<Eigen::Vector2d> observed,
                                           const std::vector<Eigen::Matrix2d> observed_cov,
                                           const std::vector<Eigen::Vector3d> ref_point,
                                           const traact::vision::CameraCalibration intrinsics) {


            return (new ceres::AutoDiffCostFunction< CeresRefNPointReprojectionError, N*2, 7 >(
                    new CeresRefNPointReprojectionError<N>(observed, observed_cov,
                                                        ref_point,intrinsics)));
        }

        const std::vector<Eigen::Vector2d> observed_;
        const std::vector<Eigen::Matrix2d> observed_cov_;

        std::vector<std::vector<double>> stddev_;

        const std::vector<Eigen::Vector3d> ref_points_;

        const double fx;
        const double fy;

        const double cx;
        const double cy;

        traact::vision::CameraCalibration m_intrinsics;
    };
}


#endif //TRAACTMULTI_CERESREFNPOINTEPREROJECTIONERROR_H

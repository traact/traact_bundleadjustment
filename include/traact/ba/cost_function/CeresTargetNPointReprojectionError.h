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

#ifndef TRAACTMULTI_CERESTARGETNPOINTEPREROJECTIONERROR_H
#define TRAACTMULTI_CERESTARGETNPOINTEPREROJECTIONERROR_H

#include "CostFunctionUtils.h"

namespace traact::ba {


    template<uint16_t N>
    struct CeresTargetNPointReprojectionError {
        CeresTargetNPointReprojectionError(const spatial::Position2DList observed,
                                        const std::vector<Eigen::Matrix2d> observed_cov,
                                        const vision::CameraCalibration &mIntrinsics) : observed_(observed),
                                                                                        observed_cov_(observed_cov),
                                                                                        m_intrinsics(mIntrinsics)
                , fx(mIntrinsics.fx), fy(mIntrinsics.fy)
                , cx(mIntrinsics.cx), cy(mIntrinsics.cy){

            stddev_.resize(N, std::vector<double>(2,1));

            for(int i=0;i<N;++i){
                stddev_[i][0] = 1;//sqrt(observed_cov_[i](0,0));
                stddev_[i][1] = 1;//sqrt(observed_cov_[i](1,1));
            }

        }

        template <typename T>
        bool operator()(const T* const camera,
                        const T* const point,
                        T* residuals) const {



            T p2d[N][2];

            for(int i=0;i<N;i++){
                const T* p = point+i*3;
                reprojectPoint(camera, p, fx,fy,cx,cy, p2d[i]);
            }


            for(int i=0;i<N;i++){
                // The error is the difference between the predicted and observed position.
                residuals[i*2+0] = (observed_[i][0] - p2d[i][0]);// / T(stddev_[i][0]);
                residuals[i*2+1] = (observed_[i][1] - p2d[i][1]);// / T(stddev_[i][1]);
            }

//            {
//                std::cout << "CeresTargetNPointReprojectionError\n";
//                for(int i=0;i<N;i++){
//                    // The error is the difference between the predicted and observed position.
//                    //std::cout << "observed_x " << observed_[i][0] << " predicted_x " << p2d[i][0]  << "\n";
//                    //std::cout << "observed_y " << observed_[i][1] << " predicted_y " << p2d[i][1]  << "\n";
//                    std::cout << "residuals " << residuals[i*2+0] << " " << residuals[i*2+1] << std::endl;
//                }
//            }

            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const spatial::Position2DList observed,
                                           const std::vector<Eigen::Matrix2d> observed_cov,
                                           const traact::vision::CameraCalibration intrinsics) {


            return (new ceres::AutoDiffCostFunction< CeresTargetNPointReprojectionError, N*2, 7, N*3 >(
                    new CeresTargetNPointReprojectionError<N>(observed, observed_cov,intrinsics)));
        }

        const spatial::Position2DList  observed_;
        const std::vector<Eigen::Matrix2d> observed_cov_;

        std::vector<std::vector<double>> stddev_;

        const double fx;
        const double fy;

        const double cx;
        const double cy;

        traact::vision::CameraCalibration m_intrinsics;
    };
}


#endif //TRAACTMULTI_CERESTARGETNPOINTEPREROJECTIONERROR_H

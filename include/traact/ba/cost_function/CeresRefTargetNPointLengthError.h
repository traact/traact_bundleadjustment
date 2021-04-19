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

#ifndef TRAACTMULTI_CERESREFTARGETNPOINTLENGTHERROR_H
#define TRAACTMULTI_CERESREFTARGETNPOINTLENGTHERROR_H


#include <traact/spatial.h>
#include "CostFunctionUtils.h"

namespace traact::ba {


    template<uint16_t N>
    struct CeresRefTargetNPointLengthError {
        CeresRefTargetNPointLengthError(const spatial::Position3DList target_model, double target_stddev)
                : observed_(target_model),
                  target_stddev_(target_stddev){


            for(int i=0;i<N;++i){
                point_data_[i*3+0] = observed_[i].x();
                point_data_[i*3+1] = observed_[i].y();
                point_data_[i*3+2] = observed_[i].z();
            }

            calculate3DDistancesSquared(point_data_.data(), distances_.data(), N);

            for(int i=0;i<count3DDistances(N);i++){
                distances_[i] = std::sqrt(distances_[i]);
            }

        }



        template <typename T>
        bool operator()(const T* const point,
                        T* residuals) const {



            T distances[count3DDistances(N)];

            calculate3DDistancesSquared<T>(point, distances, N);


            for(int i=0;i<count3DDistances(N);i++){
                distances[i] = ceres::sqrt(distances[i]);
                residuals[i] = (T(distances_[i]) - distances[i]) / T(target_stddev_);

            }

//            {
//                std::cout << "CeresRefTargetNPointLengthError resiuals:\n";
//                for(int i=0;i<count3DDistances(N);i++){
//                    std::cout << residuals[i] << " ";
//                }
//                std::cout << std::endl;
//            }

            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const spatial::Position3DList observed,
                                           const double stddev) {


            return (new ceres::AutoDiffCostFunction< CeresRefTargetNPointLengthError, count3DDistances(N), N*3 >(
                    new CeresRefTargetNPointLengthError<N>(observed, stddev)));
        }

        const spatial::Position3DList  observed_;
        const double target_stddev_;
        std::array<double, N*3> point_data_;
        std::array<double, count3DDistances(N)> distances_;


    };

    class CeresRefTargetNPointLengthErrorFactory {
    public:
        static ceres::CostFunction* Create(const spatial::Position3DList& observed,
                                           const double stddev) {

            std::size_t point_count = observed.size();
            switch (point_count) {
                case 2:
                    return CeresRefTargetNPointLengthError<2>::Create(observed, stddev);
                case 3:
                    return CeresRefTargetNPointLengthError<3>::Create(observed, stddev);
                case 4:
                    return CeresRefTargetNPointLengthError<4>::Create(observed, stddev);
                case 5:
                    return CeresRefTargetNPointLengthError<5>::Create(observed, stddev);
                case 6:
                    return CeresRefTargetNPointLengthError<6>::Create(observed, stddev);
                case 7:
                    return CeresRefTargetNPointLengthError<7>::Create(observed, stddev);
                case 8:
                    return CeresRefTargetNPointLengthError<8>::Create(observed, stddev);
//                case 9:
//                    return CeresRefTargetNPointLengthError<9>::Create(observed, stddev);
//                case 10:
//                    return CeresRefTargetNPointLengthError<10>::Create(observed, stddev);
//                case 11:
//                    return CeresRefTargetNPointLengthError<11>::Create(observed, stddev);
//                case 12:
//                    return CeresRefTargetNPointLengthError<12>::Create(observed, stddev);
//                case 13:
//                    return CeresRefTargetNPointLengthError<13>::Create(observed, stddev);
//                case 14:
//                    return CeresRefTargetNPointLengthError<14>::Create(observed, stddev);
//                case 15:
//                    return CeresRefTargetNPointLengthError<15>::Create(observed, stddev);
//                case 16:
//                    return CeresRefTargetNPointLengthError<16>::Create(observed, stddev);
                case 0:
                default:
                    spdlog::error("unsupported number of observations for CeresRefTargetNPointLengthError");
                    return nullptr;

            }


        }
    };
}

#endif //TRAACTMULTI_CERESREFTARGETNPOINTLENGTHERROR_H

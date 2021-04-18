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

#ifndef TRAACTMULTI_CERESWANDLENGTHERROR_H
#define TRAACTMULTI_CERESWANDLENGTHERROR_H

#include <traact/vision_datatypes.h>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <Eigen/Eigen>
#include <Eigen/Dense>
#include <cmath>
#include <sstream>

namespace traact::ba {
    struct CeresWandLengthError {
        CeresWandLengthError(const double wandLength,const double stddev)
                : wandLength_(wandLength), stddev_(stddev)

        {


        }

        template <typename T>
        bool operator()(const T* const p1, const T* const mutable_length,
                        T* residuals) const {


            //Eigen::Vector3d p3d_1((double)p1[0],(double)p1[1],(double)p1[2]);
            //Eigen::Vector3d p3d_2((double)p1[3],(double)p1[4],(double)p1[5]);

            //double localLength = (p3d_1 - p3d_2).norm();
            T dx = p1[0] - p1[3];
            T dy = p1[1] - p1[4];
            T dz = p1[2] - p1[5];


            T localLength = sqrt(dx*dx+dy*dy+dz*dz);

            T localDiff = mutable_length[0] - localLength;


            residuals[0] = localDiff / T(stddev_);


            return true;
        }

        // Factory to hide the construction of the CostFunction object from
        // the client code.
        static ceres::CostFunction* Create(const double wandLength,const double stddev) {


            return (new ceres::AutoDiffCostFunction< CeresWandLengthError, 1, 6,1 >(
                    new CeresWandLengthError(wandLength,stddev)));
        }



        double stddev_;
        double wandLength_;

    };
}

#endif //TRAACTMULTI_CERESWANDLENGTHERROR_H

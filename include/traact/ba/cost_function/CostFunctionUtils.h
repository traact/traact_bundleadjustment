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

    constexpr std::size_t factorial(std::size_t n)
    {
        return n <= 1 ? 1 : (n * factorial(n - 1));
    }

    constexpr std::size_t count3DDistances(std::size_t n){
        return n <= 1 ? 0 : (n-1 + count3DDistances(n - 1));
    }

    template<typename T>
    void calculate3DDistancesSquared(const T* const point_data, T* result, std::size_t N) {

        if(N < 2)
            return;

        T x0 = point_data[0];
        T y0 = point_data[1];
        T z0 = point_data[2];
        for(std::size_t i=1;i<N;++i) {
            T xn = point_data[i*3+0];
            T yn = point_data[i*3+1];
            T zn = point_data[i*3+2];

            T xd = x0 - xn;
            T yd = y0 - yn;
            T zd = z0 - zn;

            T d = xd*xd + yd*yd + zd*zd;

            result[i-1] = d;
        }

        calculate3DDistancesSquared<T>(point_data + 3, result + N - 1, N - 1);

    }


}

#endif //TRAACTMULTI_COSTFUNCTIONUTILS_H

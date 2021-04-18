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

#ifndef TRAACTMULTI_BUNDLEADJUSTMENT_H
#define TRAACTMULTI_BUNDLEADJUSTMENT_H

#include "BACamera.h"
#include "BATarget.h"
#include <Eigen/Geometry>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <ceres/ceres.h>

namespace traact::ba {
    class BundleAdjustment {
    public:
        typedef typename std::shared_ptr<BundleAdjustment> Ptr;

        virtual ~BundleAdjustment();

        void AddCamera(BACamera::Ptr camera);
        void SetTarget(BATarget::Ptr target);

        bool CheckData();

        bool Optimize();

        void SaveResult();

    protected:
        std::vector<BACamera::Ptr> cameras_;
        BATarget::Ptr target_;
        std::vector<TimestampType> used_ts_;
        std::size_t GetCameraMeaCount(TimestampType ts);
        std::shared_ptr<ceres::Problem> ceres_problem_;
        double *ceres_parameter_;

        bool TryEstimatePoints(TimestampType ts);
        bool TryEstimatePoint(TimestampType ts, std::size_t point_idx, Eigen::Vector3d& result);

        const std::size_t target_points_count_{4};
        const std::size_t camera_parameter_size_{7};
        const std::size_t target_parameter_size_{target_points_count_ * 3};
        double* GetCameraParameter(std::size_t idx);
        double* GetTargetParameter(std::size_t idx);


    };
}




#endif //TRAACTMULTI_BUNDLEADJUSTMENT_H

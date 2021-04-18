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

#ifndef TRAACTMULTI_TRAACTBACAMERA_H
#define TRAACTMULTI_TRAACTBACAMERA_H
#include <vector>
#include <map>
#include <Eigen/Core>
#include <traact/vision_datatypes.h>
#include <Eigen/Geometry>

namespace traact {





    class Target {
    public:
        typedef std::shared_ptr<Target> Ptr;

        bool isStatic;
        std::vector<Eigen::Vector3d> spheres;


        Target(bool isStatic, const std::vector<Eigen::Vector3d> &spheres);

        int GetCount();
    };

    struct TargetPointReference {
        Target::Ptr target;
        int point_id;
        inline Eigen::Vector3d GetPoint() {
            return target->spheres[point_id];
        }
    };

    struct TrackedPoint2D {
        typedef std::shared_ptr<TrackedPoint2D> Ptr;
        unsigned int id;
        std::map<TimestampType, Eigen::Vector2d> observations;
        bool is_static;
        bool is_initialized{false};
        void Init(double max_error_movement);
        TargetPointReference target_point;
        bool inline Has_TargetPoint() {
            return target_point.target != NULL;
        }
    };

    class TraactBACamera {
    public:
        typedef std::shared_ptr<TraactBACamera> Ptr;
        int id{-1};
        Eigen::Affine3d world2camera;
        traact::vision::CameraCalibration calibration;
        std::map<TimestampType, std::vector<Eigen::Vector2d> > measurements;
        std::map<TimestampType, std::vector<TrackedPoint2D::Ptr> > tracked_points;
        bool is_initialized{false};

        Target::Ptr root_target;
        Target::Ptr wand_target;
        double max_pixel_error_for_static_point = 1.0;

        void TrackPoints();

        bool TryPoseInit();

        void MergeStaticPoints();

    private:
        std::vector<Eigen::Vector2d> filterPoints(const std::vector<Eigen::Vector2d>& points);




    };
}




#endif //TRAACTMULTI_TRAACTBACAMERA_H

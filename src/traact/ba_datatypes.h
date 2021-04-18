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

#ifndef TRAACTMULTI_BA_DATATYPES_H
#define TRAACTMULTI_BA_DATATYPES_H

#include <Eigen/Eigen>
#include <unsupported/Eigen/MatrixFunctions>
#include <traact/datatypes.h>
#include <memory>
#include <map>
#include <vector>
#include <traact/vision.h>
#include <traact/util/Logging.h>
#include <ceres/problem.h>
#include <boost/filesystem.hpp>

namespace traact::ba {




    class CeresBAProblemBase {

    public:
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

    class SingleCameraWorld : public CeresBAProblemBase{

    public:
        void addMeasurements(std::vector<Eigen::Vector2d> points2d, std::vector<Eigen::Vector3d> points3d);
        void Init(const boost::filesystem::path &experiment_folder);

        void setCamera(const traact::vision::CameraCalibration &calibration);

        vision::CameraCalibration intrinsic_for_observation(size_t index) override;

        void setCeresProblem(ceres::Problem *problem) override;

        void CommitBA(const std::string& name);

        void WriteResultsNow(const std::string &filename);

        void reportReprojectionError();

    protected:
        std::vector<std::vector<Eigen::Vector2d> > all_points2d_;
        std::vector<std::vector<Eigen::Vector3d> > all_points3d_;
        traact::vision::CameraCalibration calibration_;
        boost::filesystem::path experiment_folder_;


    };


}
#endif //TRAACTMULTI_BA_DATATYPES_H

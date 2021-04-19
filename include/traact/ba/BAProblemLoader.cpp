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

#include <fstream>
#include <cereal/archives/json.hpp>
#include "BAProblemLoader.h"
#include <traact/cereal/CerealSpatial.h>
#include <traact/cereal/CerealVision.h>

bool missingKey(YAML::Node node, std::string key){
    if(node[key])
        return false;

    spdlog::error("config invalid: missing '{0}}' parameter",key);
    return true;
}

bool traact::ba::BAProblemLoader::LoadConfig(std::string config_file) {
    auto config = YAML::LoadFile(config_file);
    return LoadConfig(config);
}

bool traact::ba::BAProblemLoader::LoadConfig(YAML::Node config) {
    ba_ = std::make_shared<BundleAdjustment>();
    if(!config["ba"]){
        spdlog::error("config invalid: missing 'ba' root section");
        return false;
    }
    auto ba_config = config["ba"];
    if(!ba_config["cameras"]){
        spdlog::error("config invalid: missing 'cameras' root section");
        return false;
    }

    for(const auto& camera : ba_config["cameras"]){
        std::string camera_name = camera.first.as<std::string>();
        auto parameter = camera.second;

        if(missingKey(parameter, "static_position"))
            return false;
        if(missingKey(parameter, "static_rotation"))
            return false;
        if(missingKey(parameter, "intrinsic_file"))
            return false;
        if(missingKey(parameter, "extrinsic_file"))
            return false;
        if(missingKey(parameter, "measurement_file"))
            return false;
        if(missingKey(parameter, "result_file"))
            return false;


        bool static_position = parameter["static_position"].as<bool>();
        bool static_rotation = parameter["static_rotation"].as<bool>();



        std::string result_file = parameter["result_file"].as<std::string>();

        BACamera::Ptr ba_cam = std::make_shared<BACamera>(camera_name);

        ba_cam->setStaticPosition(static_position);
        ba_cam->setStaticRotation(static_rotation);
        ba_cam->setResultfile(result_file);




        {
            std::string intrinsic_file = parameter["intrinsic_file"].as<std::string>();
            std::ifstream stream;
            stream.open(intrinsic_file);
            cereal::JSONInputArchive archive(stream);
            vision::CameraCalibration data;
            archive(data);
            ba_cam->setIntrinsic(data);
        }

        {
            std::string extrinsic_file = parameter["extrinsic_file"].as<std::string>();
            std::ifstream stream;
            stream.open(extrinsic_file);
            cereal::JSONInputArchive archive(stream);
            spatial::Pose6D data;
            archive(data);
            ba_cam->setExtrinsic(data);
        }

        {
            std::string measurement_file = parameter["measurement_file"].as<std::string>();
            std::ifstream stream;
            stream.open(measurement_file);
            cereal::JSONInputArchive archive(stream);

            std::map<std::uint64_t, spatial::Position2DList> data;
            archive(data);
            for(const auto& tmp : data) {
                TimestampType ts = TimestampType::min() + TimeDurationType(tmp.first);
                ba_cam->SetMeasurement(ts, tmp.second);
            }
            ba_->AddCamera(ba_cam);
        }

        spdlog::info("Loaded Camera: \n{0}", ba_cam->toString());

    }


    if(!ba_config["target"]){
        spdlog::error("config invalid: missing 'target' root section");
        return false;
    }

    BATarget::Ptr ba_target = std::make_shared<BATarget>();

    auto target_parameter = ba_config["target"];

    {
        std::string measurement_file = target_parameter["model_file"].as<std::string>();
        std::ifstream stream;
        stream.open(measurement_file);
        cereal::JSONInputArchive archive(stream);

        spatial::Position3DList data;
        archive(data);
        ba_target->SetTargetData(data);
    }

    if(bool use_target_residual = target_parameter["use_target_residual"].as<bool>()){
        ba_target->SetUseTargetResidual(use_target_residual);
    }
    if(bool target_residual_stddev = target_parameter["target_residual_stddev"].as<double>()){
        ba_target->SetStdDev(target_residual_stddev);
    }

    spdlog::info("Loaded Target: \n{0}", ba_target->toString());

    ba_->SetTarget(ba_target);

    return true;
}

traact::ba::BundleAdjustment::Ptr traact::ba::BAProblemLoader::GetBundleAdjustment() {
    return ba_;
}

void traact::ba::BAProblemLoader::SaveResults() {

    ba_->SaveResult();


}

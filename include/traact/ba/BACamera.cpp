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

#include "BACamera.h"

bool traact::ba::BACamera::isStaticPosition() const {
    return static_position;
}

void traact::ba::BACamera::setStaticPosition(bool staticPosition) {
    static_position = staticPosition;
}

bool traact::ba::BACamera::isStaticRotation() const {
    return static_rotation;
}

void traact::ba::BACamera::setStaticRotation(bool staticRotation) {
    static_rotation = staticRotation;
}

const traact::vision::CameraCalibration &traact::ba::BACamera::getIntrinsic() const {
    return intrinsic_;
}

void traact::ba::BACamera::setIntrinsic(const traact::vision::CameraCalibration &intrinsic) {
    intrinsic_ = intrinsic;
}

const traact::spatial::Pose6D &traact::ba::BACamera::getExtrinsic() const {
    return extrinsic_;
}

void traact::ba::BACamera::setExtrinsic(const traact::spatial::Pose6D &extrinsic) {
    extrinsic_ = extrinsic;
}

const std::string &traact::ba::BACamera::getResultfile() const {
    return resultfile_;
}

void traact::ba::BACamera::setResultfile(const std::string &resultfile) {
    resultfile_ = resultfile;
}

std::string traact::ba::BACamera::toString() {
    std::stringstream ss;

    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    ss << "Name: " << name_ << std::endl;
    ss << "Static Position: " << static_position<< std::endl;
    ss << "Static Rotation: " << static_rotation<< std::endl;
    ss << "Intrinsic: " << intrinsic_<< std::endl;
    ss << "Extrinsic: " << std::endl << extrinsic_.matrix().format(CleanFmt)<< std::endl;

    ss << "Resultfile: " << resultfile_<< std::endl;
    ss << "Measurements: " << data_.size();


    return ss.str();
}

traact::ba::BACamera::BACamera(const std::string &name) : name_(name) {

}

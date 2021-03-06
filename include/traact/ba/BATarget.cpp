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

#include "BATarget.h"

void traact::ba::BATarget::SetTargetData(traact::spatial::Position3DList model_points) {
    model_ = model_points;
}

traact::spatial::Position3DList traact::ba::BATarget::GetTargetData() {
    return model_;
}

std::string traact::ba::BATarget::toString() {
    std::stringstream ss;

    Eigen::IOFormat CleanFmt(4, 0, ", ", "\n", "[", "]");

    ss << "Model Points: " << model_.size()<< std::endl;
    for(const auto& tmp : model_){
        ss << fmt::format("[{0}, {1}, {2}]",tmp.x(),tmp.y(),tmp.z()) << std::endl;
    }
    ss << "Measurements: " << data_.size();


    return ss.str();
}

void traact::ba::BATarget::SetStdDev(double stddev) {
    residual_stddev_ = stddev;

}

double traact::ba::BATarget::GetStdDev() {
    return residual_stddev_;
}

void traact::ba::BATarget::SetUseTargetResidual(bool value) {
    use_target_residual_ = value;
}

bool traact::ba::BATarget::IsUseTargetResidual() {
    return use_target_residual_;
}

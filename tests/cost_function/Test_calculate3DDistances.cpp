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


#include <gtest/gtest.h>
#include <traact/ba/cost_function/CostFunctionUtils.h>


void setPointData(double* data, std::size_t idx, double x, double y, double z){
    double* tmp = data + idx *3;
    tmp[0] = x;
    tmp[1] = y;
    tmp[2] = z;
}

TEST(calculate3DDistancesSquared, N_2) {
    using namespace traact::ba;

    const std::uint16_t N = 2;

    double data[N*3];
    double result[count3DDistances(N)];

    EXPECT_EQ(count3DDistances(N), 1);

    setPointData(data, 0, 0,0,0);
    setPointData(data, 1, 1,0,0);


    calculate3DDistancesSquared<double>(data, result, N);

    EXPECT_NEAR(result[0], 1.0,1e-9);
}

TEST(calculate3DDistancesSquared, N_3) {
    using namespace traact::ba;

    const std::uint16_t N = 3;

    double data[N*3];
    double result[count3DDistances(N)];

    EXPECT_EQ(count3DDistances(N), 3);

    setPointData(data, 0, 0,0,0);
    setPointData(data, 1, 1,0,0);
    setPointData(data, 2, 0,1,0);


    calculate3DDistancesSquared<double>(data, result, N);

    EXPECT_NEAR(result[0], 1.0,1e-9);
    EXPECT_NEAR(result[1], 1.0,1e-9);
    EXPECT_NEAR(result[2], 2.0,1e-9);
}

TEST(calculate3DDistancesSquared, N_4) {
    using namespace traact::ba;

    const std::uint16_t N = 4;

    double data[N*3];
    double result[count3DDistances(N)];

    EXPECT_EQ(count3DDistances(N), 6);

    setPointData(data, 0, 0,0,0);
    setPointData(data, 1, 1,0,0);
    setPointData(data, 2, 0,1,0);
    setPointData(data, 3, 0,0,1);


    calculate3DDistancesSquared<double>(data, result, N);

    EXPECT_NEAR(result[0], 1.0,1e-9);
    EXPECT_NEAR(result[1], 1.0,1e-9);
    EXPECT_NEAR(result[2], 1.0,1e-9);

    EXPECT_NEAR(result[3], 2.0,1e-9);
    EXPECT_NEAR(result[4], 2.0,1e-9);

    EXPECT_NEAR(result[5], 2.0,1e-9);
}
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
#include <iostream>


#include <fstream>

#include <boost/program_options.hpp>
#include <traact/util/Logging.h>
#include <signal.h>

#include <traact/ba/BAProblemLoader.h>

void ctrlC(int i) {
    spdlog::info("User requested exit (Ctrl-C).");

}


int main(int argc, char **argv) {

    using namespace traact;

    signal(SIGINT, &ctrlC);

    util::init_logging(spdlog::level::trace,false, "");

    if(argc != 2) {
        spdlog::error("missing configuration file. example in misc folder. how to use: artekmed_bundleadjustment ba_confing.yml");
        return 1;
    }
    // program options
  std::string config_file(argv[1]);

    ba::BAProblemLoader loader;
    if(!loader.LoadConfig(config_file))
        return 1;

    auto ba = loader.GetBundleAdjustment();
    if(!ba->CheckData())
        return 1;

    if(!ba->Optimize())
        return 1;

    loader.SaveResults();


    return 0;
}

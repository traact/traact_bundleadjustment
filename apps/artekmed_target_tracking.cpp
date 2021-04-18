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

#include <traact/traact.h>
#include <traact/facade/DefaultFacade.h>

#include <fstream>
#include <spdlog/sinks/stdout_color_sinks.h>

#include <traact/serialization/JsonGraphInstance.h>
#include <traact/component/facade/ApplicationSyncSink.h>

#include <boost/program_options.hpp>
#include <traact/util/Logging.h>
#include <traact/util/FileUtil.h>
#include <signal.h>
#include <traact/spatial.h>
#include <thread>

bool running = true;
traact::facade::Facade* myfacade{nullptr};

void ctrlC(int i) {
    spdlog::info("User requested exit (Ctrl-C).");
    if(myfacade)
        myfacade->stop();
}

std::string getIdxName(std::string name, int idx){
    return fmt::format("{0}_{1}", name, idx);
}

void addTracking(const traact::DefaultInstanceGraphPtr& pattern_graph_ptr, double idx, int ir2gray, const std::string& mkv_file, const std::string& result_file){
    using namespace traact::facade;
    using namespace traact;
    using namespace traact::dataflow;

    DefaultPatternInstancePtr
            source_pattern = pattern_graph_ptr->addPattern(getIdxName("source",idx),myfacade->instantiatePattern("KinectAzureSingleFilePlayer"));
    DefaultPatternInstancePtr
            undistort_pattern = pattern_graph_ptr->addPattern(getIdxName("undistorted",idx),myfacade->instantiatePattern("OpenCVUndistortImage"));
    DefaultPatternInstancePtr
            convert_pattern_6000 = pattern_graph_ptr->addPattern(getIdxName("convert_6000",idx), myfacade->instantiatePattern("OpenCvConvertImage"));
    DefaultPatternInstancePtr
            circle_tracking = pattern_graph_ptr->addPattern(getIdxName("circle_tracking",idx), myfacade->instantiatePattern("CircleTracking"));
    DefaultPatternInstancePtr
            estimate_pose = pattern_graph_ptr->addPattern(getIdxName("estimate_pose",idx), myfacade->instantiatePattern("EstimatePose"));
    DefaultPatternInstancePtr
            write_2dlist = pattern_graph_ptr->addPattern(getIdxName("write_2dList",idx), myfacade->instantiatePattern("FileRecorder_cereal_spatial:Position2DList"));

//    DefaultPatternInstancePtr
//            render_image = pattern_graph_ptr->addPattern(getIdxName("render_image",idx), myfacade->instantiatePattern("RenderImage"));
//    DefaultPatternInstancePtr
//            render_points = pattern_graph_ptr->addPattern(getIdxName("render_points",idx), myfacade->instantiatePattern("RenderPosition2DList"));
//    DefaultPatternInstancePtr
//            render_pose6D = pattern_graph_ptr->addPattern(getIdxName("render_pose6D",idx), myfacade->instantiatePattern("RenderPose6D"));



    pattern_graph_ptr->connect(getIdxName("source",idx), "output_ir", getIdxName("undistorted",idx), "input");
    pattern_graph_ptr->connect(getIdxName("source",idx), "output_calibration_ir", getIdxName("undistorted",idx), "input_calibration");

    pattern_graph_ptr->connect(getIdxName("undistorted",idx), "output", getIdxName("convert_6000",idx), "input");


    pattern_graph_ptr->connect(getIdxName("convert_6000",idx), "output", getIdxName("circle_tracking",idx), "input");


    pattern_graph_ptr->connect(getIdxName("circle_tracking",idx), "output", getIdxName("estimate_pose",idx), "input");



    pattern_graph_ptr->connect("source_target", "output", getIdxName("estimate_pose",idx), "input_model");
    pattern_graph_ptr->connect(getIdxName("undistorted",idx), "output_calibration", getIdxName("estimate_pose",idx), "input_calibration");


    pattern_graph_ptr->connect(getIdxName("estimate_pose",idx), "output_points", getIdxName("write_2dList",idx), "input");

//    pattern_graph_ptr->connect(getIdxName("convert_6000",idx), "output", getIdxName("render_image",idx), "input");
//    pattern_graph_ptr->connect(getIdxName("circle_tracking",idx), "output", getIdxName("render_points",idx), "input");
//    pattern_graph_ptr->connect(getIdxName("undistorted",idx), "output_calibration", getIdxName("render_pose6D",idx), "input_calibration");
//    pattern_graph_ptr->connect(getIdxName("estimate_pose",idx), "output", getIdxName("render_pose6D",idx), "input");

    source_pattern->pattern_pointer.parameter["file"]["value"] = mkv_file;
    convert_pattern_6000->pattern_pointer.parameter["irToGray"]["value"] = ir2gray;
    estimate_pose->pattern_pointer.parameter["forceZFaceCamera"]["value"] = false;
//    render_points->pattern_pointer.parameter["window"]["value"] = getIdxName("render_image",idx);
//    render_pose6D->pattern_pointer.parameter["window"]["value"] = getIdxName("render_image",idx);

    write_2dlist->pattern_pointer.parameter["file"]["value"] = result_file;

}


int main(int argc, char **argv) {

    using namespace traact::facade;
    using namespace traact;
    using namespace traact::dataflow;

    signal(SIGINT, &ctrlC);

    util::init_logging(spdlog::level::trace,false, "");

    // program options
    std::string points_file;
    std::string root_folder;
    std::string result_filename;
    std::vector<double> gray_factor;
    try {
        //describe program options
        namespace po = boost::program_options;
        po::options_description poDesc("Allowed options", 80);
        poDesc.add_options()
                ("help", "print this help message")
                ("points_file", po::value<std::string>(&points_file), "File describing the marker target")
                ("root_folder", po::value<std::string>(&root_folder), "root folder containing folders with k4a_capture.mkv files")
                ("result_filename", po::value<std::string>(&result_filename), "Result file")
                ("gray_factor", po::value<std::vector<double> >()->multitoken(), "16Bit to 8 Bit gray factor for each folder, alphabetical order")
                ;


        po::positional_options_description inputOptions;
        inputOptions.add("root_folder", 1);
        inputOptions.add("points_file", 1);
        inputOptions.add("result_filename", 1);
        inputOptions.add("gray_factor", 1);

        // parse options from command line and environment
        po::variables_map poOptions;
        po::store(po::command_line_parser(argc, argv).options(poDesc).positional(inputOptions).run(), poOptions);
        po::notify(poOptions);


        // print help message if nothing specified
        if (poOptions.count("help") || points_file.empty() || root_folder.empty() || result_filename.empty()) {
            std::cout << "Syntax: artekmed_target_tracking --points_file <points_file.json> --root_folder <root_folder.json> --result_filename <result.json>" << std::endl << std::endl;
            std::cout << poDesc << std::endl;
            return 1;
        }

        if (!poOptions["gray_factor"].empty()) {
            gray_factor = poOptions["gray_factor"].as<std::vector<double> >();

        }
    }
    catch (std::exception &e) {
        std::cerr << "Error parsing command line parameters : " << e.what() << std::endl;
        std::cerr << "Try traact_artekmed --help for help" << std::endl;
        return 1;
    }

    spdlog::info("start traact with options:");
    spdlog::info("Root folder: {0}", root_folder);
    spdlog::info("Points: {0}", points_file);
    spdlog::info("Result: {0}", result_filename);



    myfacade = new traact::facade::DefaultFacade();


    DefaultInstanceGraphPtr pattern_graph_ptr = std::make_shared<DefaultInstanceGraph>("tracking");
    DefaultPatternInstancePtr
            source_target_pattern = pattern_graph_ptr->addPattern("source_target",myfacade->instantiatePattern("FileReader_cereal_spatial:Position3DList"));
    source_target_pattern->pattern_pointer.parameter["file"]["value"] = points_file;

    std::vector<std::string> folders = traact::util::glob_dirs(root_folder);

//    folders.push_back("/media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Calib/cn01/");
//    folders.push_back("/media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Calib/cn02/");
//    folders.push_back("/media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Calib/cn03/");
//    folders.push_back("/media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Calib/cn04/");
//    folders.push_back("/media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Calib/cn05/");
//    folders.push_back("/media/frieder/System/data/inm_ba/recording_20210408_ARTCalib_Calib/cn06/");

    if(folders.size() < 1){
        spdlog::error("at least one directory with a k4a_capture.mkv file must be present");
        return -1;
    }

    if(folders.size() != gray_factor.size()) {
        spdlog::error("count of gray_factor parameters ({1}) must match directory count ({0})",folders.size() , gray_factor.size());
        return -1;
    }

    for(int i=0;i<folders.size();++i){
        std::string folder = folders[i];
        double gf = gray_factor[i];
        addTracking(pattern_graph_ptr, i, gf, fmt::format("{0}/{1}", folder, "k4a_capture.mkv"), fmt::format("{0}/{1}", folder, "LTarget_points.txt"));
    }


    buffer::TimeDomainManagerConfig td_config;
    td_config.time_domain = 0;
    td_config.ringbuffer_size = 3;
    td_config.master_source = "source_0";
    td_config.source_mode = SourceMode::WaitForBuffer;
    td_config.missing_source_event_mode = MissingSourceEventMode::WaitForEvent;
    td_config.max_offset = std::chrono::milliseconds(10);
    td_config.max_delay = std::chrono::milliseconds(100);
    td_config.measurement_delta = std::chrono::nanoseconds(33333333);

    pattern_graph_ptr->timedomain_configs[0] = td_config;




    std::string filename = pattern_graph_ptr->name + ".json";
    {
        nlohmann::json jsongraph;
        ns::to_json(jsongraph, *pattern_graph_ptr);

        std::ofstream myfile;
        myfile.open(filename);
        myfile << jsongraph.dump(4);
        myfile.close();

        //std::cout << jsongraph.dump(4) << std::endl;
    }

    myfacade->loadDataflow(pattern_graph_ptr);

    myfacade->start();
    while(running){
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    }
    myfacade->stop();

    delete myfacade;

    spdlog::info("exit program");


    return 0;
}

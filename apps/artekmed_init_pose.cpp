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
#include <signal.h>
#include <traact/spatial.h>
#include <thread>

#include <thread>

traact::facade::Facade* myfacade{nullptr};

void ctrlC(int i) {
    spdlog::info("User requested exit (Ctrl-C).");
    if(myfacade)
        myfacade->stop();
}


int main(int argc, char **argv) {

    using namespace traact::facade;
    using namespace traact;
    using namespace traact::dataflow;



    signal(SIGINT, &ctrlC);

    util::init_logging(spdlog::level::trace,false, "");

    // program options
  std::string points_file;
  std::string video_file;
  std::string result_file;
    bool debug_render = false;
  std::string result_intr_file = "";
  std::string result_raw_c2w_file = "";
  double gray_factor = 2500;
  try {
     //describe program options
    namespace po = boost::program_options;
    po::options_description poDesc("Allowed options", 80);
    poDesc.add_options()
        ("help", "print this help message")
        ("points_file", po::value<std::string>(&points_file), "File describing the marker target")
        ("video_file", po::value<std::string>(&video_file), "Kinect mkv file")
            ("debug_render", po::value<bool>(&debug_render), "Show Debug Render")
        ("result_file", po::value<std::string>(&result_file), "Result file, default orientation as used by Konsul Artekmed")
            ("result_raw", po::value<std::string>(&result_raw_c2w_file), "Result file for raw camera to target, opencv result")
            ("result_intr_file", po::value<std::string>(&result_intr_file), "Result intrinsics file")
            ("gray_factor", po::value<double>(&gray_factor), "16Bit to 8 Bit gray factor")
        ;


      po::positional_options_description inputOptions;
      inputOptions.add("video_file", 1);
      inputOptions.add("points_file", 1);
      inputOptions.add("result_file", 1);
      inputOptions.add("gray_factor", 1);
      inputOptions.add("debug_render", 1);
      inputOptions.add("result_intr_file", 0);
      inputOptions.add("result_raw_c2w_file", 0);

    // parse options from command line and environment
    po::variables_map poOptions;
    po::store(po::command_line_parser(argc, argv).options(poDesc).positional(inputOptions).run(), poOptions);
    po::notify(poOptions);


    // print help message if nothing specified
    if (poOptions.count("help") || points_file.empty()|| video_file.empty()|| result_file.empty()) {
      std::cout << "Syntax: artekmed_init_pose --points_file <points_file.json> --video_file <video_file.json> --result_file <result.json> --result_intr_file <result.json>" << std::endl << std::endl;
      std::cout << poDesc << std::endl;
      return 1;
    }
  }
  catch (std::exception &e) {
    std::cerr << "Error parsing command line parameters : " << e.what() << std::endl;
    std::cerr << "Try traact_artekmed --help for help" << std::endl;
    return 1;
  }

  spdlog::info("start traact with options:");
  spdlog::info("Video: {0}", video_file);
    spdlog::info("Points: {0}", points_file);
    spdlog::info("Result: {0}", result_file);
    spdlog::info("Result Intrinsics: {0}", result_intr_file);
    spdlog::info("Gray factor: {0}", gray_factor);




  myfacade = new traact::facade::DefaultFacade();


    DefaultInstanceGraphPtr pattern_graph_ptr = std::make_shared<DefaultInstanceGraph>("init_pose");
    DefaultPatternInstancePtr
            source_pattern = pattern_graph_ptr->addPattern("source",myfacade->instantiatePattern("KinectAzureSingleFilePlayer"));
    DefaultPatternInstancePtr
            source_target_pattern = pattern_graph_ptr->addPattern("source_target",myfacade->instantiatePattern("FileReader_cereal_spatial:Position3DList"));
    DefaultPatternInstancePtr
            undistort_pattern = pattern_graph_ptr->addPattern("undistorted",myfacade->instantiatePattern("OpenCVUndistortImage"));
    DefaultPatternInstancePtr
            convert_pattern_6000 = pattern_graph_ptr->addPattern("convert_6000", myfacade->instantiatePattern("OpenCvConvertImage"));

    DefaultPatternInstancePtr
            circle_tracking = pattern_graph_ptr->addPattern("circle_tracking", myfacade->instantiatePattern("CircleTracking"));
    DefaultPatternInstancePtr
            estimate_pose = pattern_graph_ptr->addPattern("estimate_pose", myfacade->instantiatePattern("EstimatePose"));


    DefaultPatternInstancePtr
            print_pose = pattern_graph_ptr->addPattern("print_pose", myfacade->instantiatePattern("Pose6DPrint"));
    DefaultPatternInstancePtr
            write_pose = pattern_graph_ptr->addPattern("write_pose", myfacade->instantiatePattern("FileWriter_cereal_spatial:Pose6D"));


    DefaultPatternInstancePtr
            rotate_to_unity = pattern_graph_ptr->addPattern("rotate_to_unity", myfacade->instantiatePattern("StaticPose"));
    DefaultPatternInstancePtr
            opencv_to_opengl = pattern_graph_ptr->addPattern("opencv_to_opengl", myfacade->instantiatePattern("StaticPose"));

    DefaultPatternInstancePtr
            inv_camera_to_world = pattern_graph_ptr->addPattern("inv_camera_to_world", myfacade->instantiatePattern("InversionComponent"));

    DefaultPatternInstancePtr
            mul_target_to_unity = pattern_graph_ptr->addPattern("mul_target_to_unity", myfacade->instantiatePattern("MultiplicationComponent"));
    DefaultPatternInstancePtr
            mul_opencv_to_opengl = pattern_graph_ptr->addPattern("mul_opencv_to_opengl", myfacade->instantiatePattern("MultiplicationComponent"));



    pattern_graph_ptr->connect("source", "output_ir", "undistorted", "input");
    pattern_graph_ptr->connect("source", "output_calibration_ir", "undistorted", "input_calibration");
    pattern_graph_ptr->connect("undistorted", "output", "convert_6000", "input");


    pattern_graph_ptr->connect("convert_6000", "output", "circle_tracking", "input");
    pattern_graph_ptr->connect("circle_tracking", "output", "estimate_pose", "input");
    pattern_graph_ptr->connect("source_target", "output", "estimate_pose", "input_model");
    pattern_graph_ptr->connect("undistorted", "output_calibration", "estimate_pose", "input_calibration");




    


    pattern_graph_ptr->connect("rotate_to_unity", "output", "mul_target_to_unity", "input0");

    pattern_graph_ptr->connect("estimate_pose", "output", "inv_camera_to_world", "input");
    pattern_graph_ptr->connect("inv_camera_to_world", "output", "mul_target_to_unity", "input1");

    pattern_graph_ptr->connect("mul_target_to_unity", "output", "mul_opencv_to_opengl", "input0");
    pattern_graph_ptr->connect("opencv_to_opengl", "output", "mul_opencv_to_opengl", "input1");

    pattern_graph_ptr->connect("mul_opencv_to_opengl", "output", "write_pose", "input");
    pattern_graph_ptr->connect("mul_opencv_to_opengl", "output", "print_pose", "input");


    //std::string file_path = "/media/frieder/System/data/inm_ba/ir05_withTarget/cn03/";

    if(debug_render){
        DefaultPatternInstancePtr sink_pattern = pattern_graph_ptr->addPattern("sink", myfacade->instantiatePattern("DebugWindow"));

        pattern_graph_ptr->connect("convert_6000", "output", "sink", "input");
        pattern_graph_ptr->connect("undistorted", "output_calibration", "sink", "input_intrinsics");
        pattern_graph_ptr->connect("estimate_pose", "output", "sink", "target_pose");
        pattern_graph_ptr->connect("circle_tracking", "output", "sink", "input_2d_tracking");

        sink_pattern->pattern_pointer.parameter["WaitForNextFrame"]["value"] = false;
    }


    source_pattern->pattern_pointer.parameter["file"]["value"] = video_file;// fmt::format("{0}{1}",file_path, "k4a_capture.mkv");
    source_target_pattern->pattern_pointer.parameter["file"]["value"] = points_file;// "/media/frieder/System/data/inm_ba/LTarget.json";
    write_pose->pattern_pointer.parameter["file"]["value"] = result_file;// fmt::format("{0}{1}",file_path,"world2camera.json");

    convert_pattern_6000->pattern_pointer.parameter["alpha"]["value"] = 255./gray_factor;
    convert_pattern_6000->pattern_pointer.parameter["beta"]["value"] = 0;
    estimate_pose->pattern_pointer.parameter["forceZFaceCamera"]["value"] = true;
    

    opencv_to_opengl->pattern_pointer.parameter["rx"]["value"] = 1;
    opencv_to_opengl->pattern_pointer.parameter["ry"]["value"] = 0;
    opencv_to_opengl->pattern_pointer.parameter["rz"]["value"] = 0;
    opencv_to_opengl->pattern_pointer.parameter["rw"]["value"] = 0;


    Eigen::Quaterniond rot_to_unity = Eigen::Quaterniond(0.7071,-0.7071,0,0) * Eigen::Quaterniond(0.7071,0,0,0.7071);

    rotate_to_unity->pattern_pointer.parameter["rx"]["value"] = rot_to_unity.x();
    rotate_to_unity->pattern_pointer.parameter["ry"]["value"] = rot_to_unity.y();
    rotate_to_unity->pattern_pointer.parameter["rz"]["value"] = rot_to_unity.z();
    rotate_to_unity->pattern_pointer.parameter["rw"]["value"] = rot_to_unity.w();

    if(!result_intr_file.empty()) {
        DefaultPatternInstancePtr
                write_calib = pattern_graph_ptr->addPattern("write_calib", myfacade->instantiatePattern("FileWriter_cereal_vision:CameraCalibration"));
        pattern_graph_ptr->connect("undistorted", "output_calibration", "write_calib", "input");
        write_calib->pattern_pointer.parameter["file"]["value"] = result_intr_file;
    }

    if(!result_raw_c2w_file.empty()) {
        DefaultPatternInstancePtr
                write_raw_calib = pattern_graph_ptr->addPattern("write_raw_calib", myfacade->instantiatePattern("FileWriter_cereal_spatial:Pose6D"));
        pattern_graph_ptr->connect("estimate_pose", "output", "write_raw_calib", "input");
        write_raw_calib->pattern_pointer.parameter["file"]["value"] = result_raw_c2w_file;
    }



    buffer::TimeDomainManagerConfig td_config;
    td_config.time_domain = 0;
    td_config.ringbuffer_size = 10;
    td_config.master_source = "source";
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

    myfacade->blockingStart();

    delete myfacade;

    spdlog::info("exit program");


    return 0;
}

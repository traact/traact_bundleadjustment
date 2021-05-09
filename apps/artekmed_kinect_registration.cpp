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
#include <traact/traact.h>
#include <traact/facade/DefaultFacade.h>
#include <yaml-cpp/yaml.h>
#include <traact/util/YAMLUtil.h>
#include <cppfs/FileHandle.h>
#include <cppfs/fs.h>
#include <cppfs/FilePath.h>
#include <traact/util/FileUtil.h>

traact::facade::Facade* global_facade{nullptr};
bool running{true};

void ctrlC(int i) {
    spdlog::info("User requested exit (Ctrl-C).");
    running = false;
    if(global_facade)
        global_facade->stop();
}

void calcAlphaBeta(double threshold, double min, double max, double& alpha, double& beta){
    threshold = 0;//threshold / 2;
    alpha = (threshold - 255) / (min - max);
    beta = 255 - max*alpha;
}

std::string getIdxName(std::string name, int idx){
    return fmt::format("{0}_{1}", name, idx);
}

bool addCameraProcessing(const traact::DefaultInstanceGraphPtr &pattern_graph_ptr, double idx, YAML::Node& parameter, YAML::Node& default_parameter, const std::string& mkv_file, bool debug_render, bool forceZUp) {
    using namespace traact::facade;
    using namespace traact;
    using namespace traact::dataflow;


    double gray_threshold,gray_min,gray_max, area_min, area_max;
    bool filter_area;
    std::int64_t stop_after_n_frames{-1l};

    if(!util::SetValue(gray_threshold, "gray_threshold", parameter["image_16To8Bit"], default_parameter["image_16To8Bit"]))
        return false;
    if(!util::SetValue(gray_min, "gray_min", parameter["image_16To8Bit"], default_parameter["image_16To8Bit"]))
        return false;
    if(!util::SetValue(gray_max, "gray_max", parameter["image_16To8Bit"], default_parameter["image_16To8Bit"]))
        return false;

    if(!util::SetValue(filter_area, "filter_area", parameter["circle_tracking"], default_parameter["circle_tracking"]))
        return false;
    if(filter_area) {
        if(!util::SetValue(area_min, "area_min", parameter["circle_tracking"], default_parameter["circle_tracking"]))
            return false;
        if(!util::SetValue(area_max, "area_max", parameter["circle_tracking"], default_parameter["circle_tracking"]))
            return false;
    }
    util::SetValue(stop_after_n_frames, "stop_after_n_frames", parameter["playback"], default_parameter["playback"]);


    double cvt_alpha, cvt_beta;
    calcAlphaBeta(gray_threshold, gray_min, gray_max, cvt_alpha, cvt_beta);

    DefaultPatternInstancePtr
            source_pattern = pattern_graph_ptr->addPattern(getIdxName("source",idx),global_facade->instantiatePattern("KinectAzureSingleFilePlayer"));
    DefaultPatternInstancePtr
            undistort_pattern = pattern_graph_ptr->addPattern(getIdxName("undistorted",idx),global_facade->instantiatePattern("OpenCVUndistortImage"));
    DefaultPatternInstancePtr
            convert_pattern_6000 = pattern_graph_ptr->addPattern(getIdxName("convert_6000",idx), global_facade->instantiatePattern("OpenCvConvertImage"));
    DefaultPatternInstancePtr
            circle_tracking = pattern_graph_ptr->addPattern(getIdxName("circle_tracking",idx), global_facade->instantiatePattern("CircleTracking"));
    DefaultPatternInstancePtr
            estimate_pose = pattern_graph_ptr->addPattern(getIdxName("estimate_pose",idx), global_facade->instantiatePattern("EstimatePose"));


    pattern_graph_ptr->connect(getIdxName("source",idx), "output_ir", getIdxName("undistorted",idx), "input");
    pattern_graph_ptr->connect(getIdxName("source",idx), "output_calibration_ir", getIdxName("undistorted",idx), "input_calibration");

    pattern_graph_ptr->connect(getIdxName("undistorted",idx), "output", getIdxName("convert_6000",idx), "input");

    pattern_graph_ptr->connect(getIdxName("convert_6000",idx), "output", getIdxName("circle_tracking",idx), "input");

    pattern_graph_ptr->connect(getIdxName("circle_tracking",idx), "output", getIdxName("estimate_pose",idx), "input");
    pattern_graph_ptr->connect("source_target", "output", getIdxName("estimate_pose",idx), "input_model");
    pattern_graph_ptr->connect(getIdxName("undistorted",idx), "output_calibration", getIdxName("estimate_pose",idx), "input_calibration");



    if(debug_render){
        DefaultPatternInstancePtr render_image = pattern_graph_ptr->addPattern(getIdxName("render_image",idx), global_facade->instantiatePattern("RenderImage"));
        DefaultPatternInstancePtr render_points = pattern_graph_ptr->addPattern(getIdxName("render_points",idx), global_facade->instantiatePattern("RenderPosition2DList"));
        DefaultPatternInstancePtr render_pose6D = pattern_graph_ptr->addPattern(getIdxName("render_pose6D",idx), global_facade->instantiatePattern("RenderPose6D"));

        pattern_graph_ptr->connect(getIdxName("convert_6000",idx), "output", getIdxName("render_image",idx), "input");
        pattern_graph_ptr->connect(getIdxName("circle_tracking",idx), "output", getIdxName("render_points",idx), "input");
        pattern_graph_ptr->connect(getIdxName("undistorted",idx), "output_calibration", getIdxName("render_pose6D",idx), "input_calibration");
        pattern_graph_ptr->connect(getIdxName("estimate_pose",idx), "output", getIdxName("render_pose6D",idx), "input");


        render_points->pattern_pointer.parameter["window"]["value"] = getIdxName("render_image",idx);
        render_pose6D->pattern_pointer.parameter["window"]["value"] = getIdxName("render_image",idx);
    }


    source_pattern->pattern_pointer.parameter["file"]["value"] = mkv_file;
    source_pattern->pattern_pointer.parameter["stop_after_n_frames"]["value"] = stop_after_n_frames;
    convert_pattern_6000->pattern_pointer.parameter["alpha"]["value"] = cvt_alpha;
    convert_pattern_6000->pattern_pointer.parameter["beta"]["value"] = cvt_beta;
    circle_tracking->pattern_pointer.parameter["threshold"]["value"] = gray_threshold;
    circle_tracking->pattern_pointer.parameter["filter_area"]["value"] = filter_area;
    if(filter_area) {
        circle_tracking->pattern_pointer.parameter["area_min"]["value"] = area_min;
        circle_tracking->pattern_pointer.parameter["area_max"]["value"] = area_max;
    }
    estimate_pose->pattern_pointer.parameter["forceZFaceCamera"]["value"] = forceZUp;

    return true;
}

bool addCameraProcessingDistorted(const traact::DefaultInstanceGraphPtr &pattern_graph_ptr, double idx, YAML::Node& parameter, YAML::Node& default_parameter, const std::string& mkv_file, bool debug_render, bool forceZUp) {
    using namespace traact::facade;
    using namespace traact;
    using namespace traact::dataflow;


    double gray_threshold,gray_min,gray_max, area_min, area_max;
    bool filter_area;
    std::int64_t stop_after_n_frames{-1l};

    if(!util::SetValue(gray_threshold, "gray_threshold", parameter["image_16To8Bit"], default_parameter["image_16To8Bit"]))
        return false;
    if(!util::SetValue(gray_min, "gray_min", parameter["image_16To8Bit"], default_parameter["image_16To8Bit"]))
        return false;
    if(!util::SetValue(gray_max, "gray_max", parameter["image_16To8Bit"], default_parameter["image_16To8Bit"]))
        return false;

    if(!util::SetValue(filter_area, "filter_area", parameter["circle_tracking"], default_parameter["circle_tracking"]))
        return false;
    if(filter_area) {
        if(!util::SetValue(area_min, "area_min", parameter["circle_tracking"], default_parameter["circle_tracking"]))
            return false;
        if(!util::SetValue(area_max, "area_max", parameter["circle_tracking"], default_parameter["circle_tracking"]))
            return false;
    }
    util::SetValue(stop_after_n_frames, "stop_after_n_frames", parameter["playback"], default_parameter["playback"]);


    double cvt_alpha, cvt_beta;
    calcAlphaBeta(gray_threshold, gray_min, gray_max, cvt_alpha, cvt_beta);

    DefaultPatternInstancePtr
            source_pattern = pattern_graph_ptr->addPattern(getIdxName("source",idx),global_facade->instantiatePattern("KinectAzureSingleFilePlayer"));
    //DefaultPatternInstancePtr
    //        undistort_pattern = pattern_graph_ptr->addPattern(getIdxName("undistorted",idx),global_facade->instantiatePattern("OpenCVUndistortImage"));
    DefaultPatternInstancePtr
            convert_pattern_6000 = pattern_graph_ptr->addPattern(getIdxName("convert_6000",idx), global_facade->instantiatePattern("OpenCvConvertImage"));
    DefaultPatternInstancePtr
            circle_tracking = pattern_graph_ptr->addPattern(getIdxName("circle_tracking",idx), global_facade->instantiatePattern("CircleTracking"));
    DefaultPatternInstancePtr
            estimate_pose = pattern_graph_ptr->addPattern(getIdxName("estimate_pose",idx), global_facade->instantiatePattern("EstimatePose"));


    pattern_graph_ptr->connect(getIdxName("source",idx), "output_ir", getIdxName("convert_6000",idx), "input");


    //pattern_graph_ptr->connect(getIdxName("source",idx), "output_calibration_ir", getIdxName("undistorted",idx), "input_calibration");
    //pattern_graph_ptr->connect(getIdxName("undistorted",idx), "output", getIdxName("convert_6000",idx), "input");

    pattern_graph_ptr->connect(getIdxName("convert_6000",idx), "output", getIdxName("circle_tracking",idx), "input");

    pattern_graph_ptr->connect(getIdxName("circle_tracking",idx), "output", getIdxName("estimate_pose",idx), "input");
    pattern_graph_ptr->connect(getIdxName("circle_tracking",idx), "output", getIdxName("undistorted_points",idx), "input");
    //pattern_graph_ptr->connect(getIdxName("undistorted_points",idx), "output", getIdxName("estimate_pose",idx), "input");
    pattern_graph_ptr->connect("source_target", "output", getIdxName("estimate_pose",idx), "input_model");
    pattern_graph_ptr->connect(getIdxName("source",idx), "output_calibration_ir", getIdxName("estimate_pose",idx), "input_calibration");
    //pattern_graph_ptr->connect(getIdxName("undistorted",idx), "output_calibration", getIdxName("estimate_pose",idx), "input_calibration");



    if(debug_render){
        DefaultPatternInstancePtr render_image = pattern_graph_ptr->addPattern(getIdxName("render_image",idx), global_facade->instantiatePattern("RenderImage"));
        DefaultPatternInstancePtr render_points = pattern_graph_ptr->addPattern(getIdxName("render_points",idx), global_facade->instantiatePattern("RenderPosition2DList"));
        DefaultPatternInstancePtr render_pose6D = pattern_graph_ptr->addPattern(getIdxName("render_pose6D",idx), global_facade->instantiatePattern("RenderPose6D"));

        pattern_graph_ptr->connect(getIdxName("convert_6000",idx), "output", getIdxName("render_image",idx), "input");
        pattern_graph_ptr->connect(getIdxName("circle_tracking",idx), "output", getIdxName("render_points",idx), "input");
        //pattern_graph_ptr->connect(getIdxName("undistorted",idx), "output_calibration", getIdxName("render_pose6D",idx), "input_calibration");
        pattern_graph_ptr->connect(getIdxName("source",idx), "output_calibration_ir", getIdxName("render_pose6D",idx), "input_calibration");
        pattern_graph_ptr->connect(getIdxName("estimate_pose",idx), "output", getIdxName("render_pose6D",idx), "input");


        render_points->pattern_pointer.parameter["window"]["value"] = getIdxName("render_image",idx);
        render_pose6D->pattern_pointer.parameter["window"]["value"] = getIdxName("render_image",idx);
    }


    source_pattern->pattern_pointer.parameter["file"]["value"] = mkv_file;
    source_pattern->pattern_pointer.parameter["stop_after_n_frames"]["value"] = stop_after_n_frames;
    convert_pattern_6000->pattern_pointer.parameter["alpha"]["value"] = cvt_alpha;
    convert_pattern_6000->pattern_pointer.parameter["beta"]["value"] = cvt_beta;
    circle_tracking->pattern_pointer.parameter["threshold"]["value"] = gray_threshold;
    circle_tracking->pattern_pointer.parameter["filter_area"]["value"] = filter_area;
    if(filter_area) {
        circle_tracking->pattern_pointer.parameter["area_min"]["value"] = area_min;
        circle_tracking->pattern_pointer.parameter["area_max"]["value"] = area_max;
    }
    estimate_pose->pattern_pointer.parameter["forceZFaceCamera"]["value"] = forceZUp;

    return true;
}

bool addInit(const traact::DefaultInstanceGraphPtr &pattern_graph_ptr, double idx, YAML::Node& parameter,YAML::Node& default_parameter, bool debug_render) {
    using namespace traact::facade;
    using namespace traact;
    using namespace traact::dataflow;

    if(!util::HasValue("origin_mkv_file", parameter))
        return false;
    if(!util::HasValue("result_file_init", parameter))
        return false;
    if(!util::HasValue("intrinsic_file", parameter))
        return false;
    if(!util::HasValue("extrinsic_file", parameter))
        return false;
    if(!util::HasValue("init_root_folder", default_parameter))
        return false;
    if(!util::HasValue("tracking_root_folder", default_parameter))
        return false;

    std::string init_root = default_parameter["init_root_folder"].as<std::string>();
    std::string tracking_root = default_parameter["tracking_root_folder"].as<std::string>();
    std::string mkv_file = parameter["origin_mkv_file"].as<std::string>();
    std::string result_file = parameter["result_file_init"].as<std::string>();
    std::string result_intr_file = parameter["intrinsic_file"].as<std::string>();
    std::string result_raw_c2w_file = parameter["extrinsic_file"].as<std::string>();

    cppfs::FilePath init_root_fp(init_root);
    auto mkv_file_fp = init_root_fp.resolve(mkv_file);
    cppfs::FileHandle mkv_file_fh = cppfs::fs::open(mkv_file_fp.fullPath());
    if(!mkv_file_fh.exists()) {
        spdlog::error("init mkv file missing: {0}", mkv_file_fp.fullPath());
        return false;
    }

    cppfs::FilePath tracking_root_fp(tracking_root);
    result_file = tracking_root_fp.resolve(result_file).fullPath();
    result_intr_file = tracking_root_fp.resolve(result_intr_file).fullPath();
    result_raw_c2w_file = tracking_root_fp.resolve(result_raw_c2w_file).fullPath();
    mkv_file = mkv_file_fp.fullPath();

    YAML::Node init_default_parameter = default_parameter["init"];
    if(!addCameraProcessing(pattern_graph_ptr, idx, parameter, init_default_parameter ,mkv_file, debug_render, false))
        return false;


    DefaultPatternInstancePtr
            print_pose = pattern_graph_ptr->addPattern(getIdxName("print_pose",idx), global_facade->instantiatePattern("Pose6DPrint"));
    DefaultPatternInstancePtr
            write_pose = pattern_graph_ptr->addPattern(getIdxName("write_pose",idx), global_facade->instantiatePattern("FileWriter_cereal_spatial:Pose6D"));
    DefaultPatternInstancePtr
            write_calib = pattern_graph_ptr->addPattern(getIdxName("write_calib",idx), global_facade->instantiatePattern("FileWriter_cereal_vision:CameraCalibration"));
    DefaultPatternInstancePtr
            write_raw_calib = pattern_graph_ptr->addPattern(getIdxName("write_raw_calib",idx), global_facade->instantiatePattern("FileWriter_cereal_spatial:Pose6D"));

    DefaultPatternInstancePtr
            rotate_to_unity = pattern_graph_ptr->addPattern(getIdxName("rotate_to_unity",idx), global_facade->instantiatePattern("StaticPose"));
    DefaultPatternInstancePtr
            opencv_to_opengl = pattern_graph_ptr->addPattern(getIdxName("opencv_to_opengl",idx), global_facade->instantiatePattern("StaticPose"));

    DefaultPatternInstancePtr
            inv_camera_to_world = pattern_graph_ptr->addPattern(getIdxName("inv_camera_to_world",idx), global_facade->instantiatePattern("InversionComponent"));

    DefaultPatternInstancePtr
            mul_target_to_unity = pattern_graph_ptr->addPattern(getIdxName("mul_target_to_unity",idx), global_facade->instantiatePattern("MultiplicationComponent"));
    DefaultPatternInstancePtr
            mul_opencv_to_opengl = pattern_graph_ptr->addPattern(getIdxName("mul_opencv_to_opengl",idx), global_facade->instantiatePattern("MultiplicationComponent"));


    pattern_graph_ptr->connect(getIdxName("rotate_to_unity",idx), "output", getIdxName("mul_target_to_unity",idx), "input0");

    pattern_graph_ptr->connect(getIdxName("estimate_pose",idx), "output", getIdxName("inv_camera_to_world",idx), "input");
    pattern_graph_ptr->connect(getIdxName("inv_camera_to_world",idx), "output", getIdxName("mul_target_to_unity",idx), "input1");

    pattern_graph_ptr->connect(getIdxName("mul_target_to_unity",idx), "output", getIdxName("mul_opencv_to_opengl",idx), "input0");
    pattern_graph_ptr->connect(getIdxName("opencv_to_opengl",idx), "output", getIdxName("mul_opencv_to_opengl",idx), "input1");

    //pattern_graph_ptr->connect(getIdxName("mul_opencv_to_opengl",idx), "output", getIdxName("write_pose",idx), "input");
    //pattern_graph_ptr->connect(getIdxName("mul_opencv_to_opengl",idx), "output", getIdxName("print_pose",idx), "input");


    //pattern_graph_ptr->connect(getIdxName("undistorted",idx), "output_calibration", getIdxName("write_calib",idx), "input");
    pattern_graph_ptr->connect(getIdxName("source",idx), "output_calibration_ir", getIdxName("write_calib",idx), "input");

    pattern_graph_ptr->connect(getIdxName("estimate_pose",idx), "output", getIdxName("write_raw_calib",idx), "input");

    opencv_to_opengl->pattern_pointer.parameter["rx"]["value"] = 1.0;
    opencv_to_opengl->pattern_pointer.parameter["ry"]["value"] = 0.0;
    opencv_to_opengl->pattern_pointer.parameter["rz"]["value"] = 0.0;
    opencv_to_opengl->pattern_pointer.parameter["rw"]["value"] = 0.0;


    Eigen::Quaterniond rot_to_unity = Eigen::Quaterniond(0.7071,-0.7071,0,0) * Eigen::Quaterniond(0.7071,0,0,0.7071);

    rotate_to_unity->pattern_pointer.parameter["rx"]["value"] = rot_to_unity.x();
    rotate_to_unity->pattern_pointer.parameter["ry"]["value"] = rot_to_unity.y();
    rotate_to_unity->pattern_pointer.parameter["rz"]["value"] = rot_to_unity.z();
    rotate_to_unity->pattern_pointer.parameter["rw"]["value"] = rot_to_unity.w();

    write_calib->pattern_pointer.parameter["file"]["value"] = result_intr_file;
    write_raw_calib->pattern_pointer.parameter["file"]["value"] = result_raw_c2w_file;
    write_pose->pattern_pointer.parameter["file"]["value"] = result_file;




    DefaultPatternInstancePtr
        origin2target_calib = pattern_graph_ptr->addPattern(getIdxName("origin2target_calib",idx), global_facade->instantiatePattern("InversionComponent"));

    pattern_graph_ptr->connect("read_target2origin_calib", "output", getIdxName("origin2target_calib",idx), "input");

    DefaultPatternInstancePtr
            mul_camera_to_origin = pattern_graph_ptr->addPattern(getIdxName("mul_camera_to_origin",idx), global_facade->instantiatePattern("MultiplicationComponent"));

    pattern_graph_ptr->connect(getIdxName("origin2target_calib",idx), "output", getIdxName("mul_camera_to_origin",idx), "input0");
    pattern_graph_ptr->connect(getIdxName("mul_opencv_to_opengl",idx), "output", getIdxName("mul_camera_to_origin",idx), "input1");


    pattern_graph_ptr->connect(getIdxName("mul_camera_to_origin",idx), "output", getIdxName("write_pose",idx), "input");
    pattern_graph_ptr->connect(getIdxName("mul_camera_to_origin",idx), "output", getIdxName("print_pose",idx), "input");

    return true;
}

bool addTracking(const traact::DefaultInstanceGraphPtr &pattern_graph_ptr, double idx, YAML::Node& parameter,YAML::Node& default_parameter, bool debug_render) {
    using namespace traact::facade;
    using namespace traact;
    using namespace traact::dataflow;

    if(!util::HasValue("tracking_mkv_file", parameter))
        return false;
    if(!util::HasValue("measurement_file", parameter))
        return false;
    if(!util::HasValue("tracking_root_folder", default_parameter))
        return false;

    std::string tracking_root = default_parameter["tracking_root_folder"].as<std::string>();
    std::string mkv_file = parameter["tracking_mkv_file"].as<std::string>();
    std::string result_file = parameter["measurement_file"].as<std::string>();



    cppfs::FilePath tracking_root_fp(tracking_root);
    auto mkv_file_fp = tracking_root_fp.resolve(mkv_file);
    cppfs::FileHandle mkv_file_fh = cppfs::fs::open(mkv_file_fp.fullPath());
    if(!mkv_file_fh.exists()) {
        spdlog::error("tracking mkv file missing: {0}", mkv_file_fp.fullPath());
        return false;
    }

    result_file = tracking_root_fp.resolve(result_file).fullPath();
    mkv_file = mkv_file_fp.fullPath();

    YAML::Node init_default_parameter = default_parameter["tracking"];
    if(!addCameraProcessingDistorted(pattern_graph_ptr, idx, parameter, init_default_parameter ,mkv_file, debug_render, false))
        return false;


    DefaultPatternInstancePtr
            write_2dlist = pattern_graph_ptr->addPattern(getIdxName("write_2dList",idx), global_facade->instantiatePattern("FileRecorder_cereal_spatial:Position2DList"));
    DefaultPatternInstancePtr
            undistort_points_pattern = pattern_graph_ptr->addPattern(getIdxName("undistorted_points",idx),global_facade->instantiatePattern("OpenCVUndistort2DPoints"));

    pattern_graph_ptr->connect(getIdxName("source",idx), "output_calibration_ir", getIdxName("undistorted_points",idx), "input_calibration");
    pattern_graph_ptr->connect(getIdxName("estimate_pose",idx), "output_points", getIdxName("undistorted_points",idx), "input");
    pattern_graph_ptr->connect(getIdxName("undistorted_points",idx), "output", getIdxName("write_2dList",idx), "input");


    write_2dlist->pattern_pointer.parameter["file"]["value"] = result_file;

    return true;

}

bool DoInit(YAML::Node config, bool debug_render) {
    using namespace traact::facade;
    using namespace traact;
    using namespace traact::dataflow;

    global_facade = new traact::facade::DefaultFacade();

    if(!util::HasValue("ba", config)) {
        return false;
    }
    auto ba_config = config["ba"];
    if(!util::HasValue("cameras", ba_config)) {
        return false;
    }
    if(!util::HasValue("target", ba_config)) {
        return false;
    }


    auto ba_cameras = ba_config["cameras"];
    auto ba_target = ba_config["target"];

    if(!util::HasValue("model_file", ba_target)) {
        return false;
    }
    if(!util::HasValue("target_to_origin_file", ba_target)) {
        return false;
    }

    std::string target_file = ba_target["model_file"].as<std::string>();
    std::string target_to_origin_file = ba_target["target_to_origin_file"].as<std::string>();
    if(!util::FileExists(target_file, "model_file")){
        return false;
    }
    if(!util::FileExists(target_to_origin_file, "target_to_origin_file")){
        return false;
    }

    if(!util::HasValue("default", ba_config)) {
        return false;
    }
    auto default_config = ba_config["default"];

    if(!util::HasValue("init", default_config)) {
        return false;
    }
    auto init_default_config = default_config["init"];



    DefaultInstanceGraphPtr pattern_graph_ptr = std::make_shared<DefaultInstanceGraph>("tracking");
    DefaultPatternInstancePtr
            source_target_pattern = pattern_graph_ptr->addPattern("source_target",global_facade->instantiatePattern("FileReader_cereal_spatial:Position3DList"));
    source_target_pattern->pattern_pointer.parameter["file"]["value"] = target_file;
    DefaultPatternInstancePtr
            read_target2origin_calib = pattern_graph_ptr->addPattern("read_target2origin_calib", global_facade->instantiatePattern("FileReader_cereal_spatial:Pose6D"));
    read_target2origin_calib->pattern_pointer.parameter["file"]["value"] = "/home/frieder/projects/traact_workspace/traact_bundleadjustment/misc/narvisTarget2charucoMarker.json";


    int camera_idx = 0;
    for(const auto& camera : ba_cameras){
        std::string camera_name = camera.first.as<std::string>();
        auto parameter = camera.second;

        if(!addInit(pattern_graph_ptr, camera_idx, parameter,default_config, debug_render))
            return false;
        camera_idx++;
    }

    buffer::TimeDomainManagerConfig td_config;
    td_config.time_domain = 0;
    td_config.ringbuffer_size = 3;
    td_config.master_source = "source_0";
    td_config.source_mode = SourceMode::WaitForBuffer;
    td_config.missing_source_event_mode = MissingSourceEventMode::WaitForEvent;
    td_config.max_offset = std::chrono::milliseconds(14);
    td_config.max_delay = std::chrono::milliseconds(100);
    td_config.measurement_delta = std::chrono::nanoseconds(33333333);

    pattern_graph_ptr->timedomain_configs[0] = td_config;

    global_facade->loadDataflow(pattern_graph_ptr);

    global_facade->blockingStart();

    delete global_facade;
    global_facade = nullptr;
    return true;
}

bool DoTracking(YAML::Node config, bool debug_render) {
    using namespace traact::facade;
    using namespace traact;
    using namespace traact::dataflow;

    global_facade = new traact::facade::DefaultFacade();



    if(!util::HasValue("ba", config)) {
        return false;
    }
    auto ba_config = config["ba"];
    if(!util::HasValue("cameras", ba_config)) {
        return false;
    }
    if(!util::HasValue("target", ba_config)) {
        return false;
    }

    auto ba_cameras = ba_config["cameras"];
    auto ba_target = ba_config["target"];

    if(!util::HasValue("model_file", ba_target)) {
        return false;
    }
    std::string target_file = ba_target["model_file"].as<std::string>();

    if(!util::FileExists(target_file, "model_file")){
        return false;
    }

    if(!util::HasValue("default", ba_config)) {
        return false;
    }
    auto default_config = ba_config["default"];

    if(!util::HasValue("tracking", default_config)) {
        return false;
    }
    auto tracking_default_config = default_config["tracking"];


    DefaultInstanceGraphPtr pattern_graph_ptr = std::make_shared<DefaultInstanceGraph>("tracking");
    DefaultPatternInstancePtr
            source_target_pattern = pattern_graph_ptr->addPattern("source_target",global_facade->instantiatePattern("FileReader_cereal_spatial:Position3DList"));
    source_target_pattern->pattern_pointer.parameter["file"]["value"] = target_file;

    int camera_idx = 0;
    for(const auto& camera : ba_cameras){
        std::string camera_name = camera.first.as<std::string>();
        auto parameter = camera.second;

        if(!addTracking(pattern_graph_ptr, camera_idx, parameter, default_config, debug_render))
            return false;
        camera_idx++;
    }


    buffer::TimeDomainManagerConfig td_config;
    td_config.time_domain = 0;
    td_config.ringbuffer_size = 3;
    td_config.master_source = "source_0";
    td_config.source_mode = SourceMode::WaitForBuffer;
    td_config.missing_source_event_mode = MissingSourceEventMode::WaitForEvent;
    td_config.max_offset = std::chrono::milliseconds(14);
    td_config.max_delay = std::chrono::milliseconds(100);
    td_config.measurement_delta = std::chrono::nanoseconds(33333333);

    pattern_graph_ptr->timedomain_configs[0] = td_config;

    global_facade->loadDataflow(pattern_graph_ptr);

    global_facade->blockingStart();

    delete global_facade;
    global_facade = nullptr;
    return true;
}

void DoBa(YAML::Node config) {
    traact::ba::BAProblemLoader loader;
    if(!loader.LoadConfig(config))
        return;

    auto ba = loader.GetBundleAdjustment();
    if(!ba->CheckData())
        return;

    if(!ba->Optimize())
        return;

    loader.SaveResults();
}

int main(int argc, char **argv) {

    using namespace traact;

    signal(SIGINT, &ctrlC);

    util::init_logging(spdlog::level::trace,false, "");

    if(argc != 2) {
        spdlog::error("missing configuration file. example in misc folder. how to use: artekmed_kinect_registration ba_confing.yml");
        return 1;
    }
    // program options
    std::string config_file(argv[1]);

    auto all_config = YAML::LoadFile(config_file);

    if(!all_config["config"]){
        spdlog::error("missing 'config' section");
    }

    auto config = all_config["config"];
    bool do_init_pose = true;
    bool do_tracking = true;
    bool do_ba = true;
    bool show_init_render = true;
    bool show_tracking_render = true;
    if(config["do_init_pose"])
        do_init_pose = config["do_init_pose"].as<bool>();
    if(config["do_tracking"])
        do_tracking = config["do_tracking"].as<bool>();
    if(config["do_ba"])
        do_ba = config["do_ba"].as<bool>();
    if(config["show_init_render"])
        show_init_render = config["show_init_render"].as<bool>();
    if(config["show_tracking_render"])
        show_tracking_render = config["show_tracking_render"].as<bool>();

    if(do_init_pose){
        if(!DoInit(all_config, show_init_render))
            return 1;
    }
    if(!running)
        return 0;
    if(do_tracking){
        if(!DoTracking(all_config, show_tracking_render))
            return 1;
    }
    if(!running)
        return 0;
    if(do_ba){
        DoBa(all_config);
    }





    return 0;
}
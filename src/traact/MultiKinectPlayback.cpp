//
// Created by frieder on 02.02.20.
//

#include "MultiKinectPlayback.h"
#include <boost/filesystem/operations.hpp>
#include <opencv2/imgproc.hpp>
#include <iostream>
#include <stdexcept>
#include <traact/util/Logging.h>

namespace traact::ba {


    MultiKinectPlayback::MultiKinectPlayback(std::string recordFolder, double irToGrayFactor, int camera_count) {
        m_irToGrayFactor = irToGrayFactor;
        master_index = -1;
        last_master_ts_ = 0;
        for(int cameraIndex = 0; cameraIndex < camera_count;cameraIndex++){
            boost::filesystem::path root_dir(recordFolder);
            boost::filesystem::path p = root_dir / (std::string("cn0")+ std::to_string(cameraIndex+1));

            spdlog::info( "reading path {0} index: {1}",p.string() ,cameraIndex);

            m_cameraFolder.push_back(p);


            recording_t localRecording;


            k4a_result_t result = K4A_RESULT_SUCCEEDED;
            std::string k4aFile = (p / "k4a_capture.mkv").string();

            result = k4a_playback_open(k4aFile.c_str(), &localRecording.handle);
            if (result != K4A_RESULT_SUCCEEDED)
                throw std::runtime_error("Could not open file \"" + k4aFile + "\".");
            result = k4a_playback_get_record_configuration(localRecording.handle, &localRecording.record_config);
            if (result != K4A_RESULT_SUCCEEDED) {
                throw std::runtime_error("Failed to get record configuration for file: \"" + k4aFile + "\".");
            }

            if (localRecording.record_config.wired_sync_mode == K4A_WIRED_SYNC_MODE_MASTER) {
                master_index = cameraIndex;
                localRecording.is_master = true;
            } else {
                localRecording.is_master = false;
            }

            localRecording.camera_index = cameraIndex;

            m_k4a_recordings.push_back(localRecording);


            //traact::vision::CameraCalibration intrinsicsDistorted = get_K4A_IRIntrinsics(&localRecording);
            auto intrinsicsDistorted = get_K4A_IRIntrinsics(&localRecording);


            std::shared_ptr<traact::vision::UndistortionHelper> undistorter(new traact::vision::UndistortionHelper());

            undistorter->Init(intrinsicsDistorted, true, false, 1);


            m_undistorter.push_back(undistorter);


        }

        advanced_frames_.resize(m_cameraFolder.size());

        // skip first 5 frames

        //advanceFrame();
        m_currentRawImages.clear();
        m_currentRawImages.resize(getCameraCount());
        m_currentUnsidtoredImages.clear();
        m_currentUnsidtoredImages.resize(getCameraCount());
        for (int i = 0; i < 5; ++i) {
            for (int cameraIndex = 0; cameraIndex < m_k4a_recordings.size(); ++cameraIndex) {
                auto &recording = m_k4a_recordings[cameraIndex];
                advanceFrameForCamera(&recording);

                // don't release last capture as it is the current one
                //if(i<4)
                //    k4a_capture_release(recording.capture);
            }
        }


    }

    size_t MultiKinectPlayback::getCameraCount() {
        return m_undistorter.size();

    }

    std::string MultiKinectPlayback::getCameraFolder(size_t index) {
        return m_cameraFolder[index].string();
    }

    std::string MultiKinectPlayback::getCameraFolderFileNAme(size_t index, std::string filename) {
        return (m_cameraFolder[index] / filename).string();
    }

    bool MultiKinectPlayback::advanceFrame() {
        for(int i=0;i<advanced_frames_.size();++i) {
            advanced_frames_[i] = 0;
        }

        std::vector<int64_t> time_diff;
        time_diff.resize(getCameraCount());

        bool reachedEnd = false;
        bool validImages = true;

        do {
            validImages = true;
            // advance master
            auto &master_recording = m_k4a_recordings[master_index];
            spdlog::debug("advance master");
            spdlog::debug("--------------------------------------------------------");
            if(!advance_master())
                return false;

            uint64_t current_master_ts = master_recording.current_ts;

            //#pragma omp parallel for //default(none) shared(reachedEnd, m_currentRawImages, m_currentUnsidtoredImages,m_lastTimeStamps,validImage, std::cout)
            for (int camera_index = 0; camera_index < m_k4a_recordings.size(); ++camera_index) {
                if(camera_index == master_index) {
                    time_diff[camera_index] = 0;
                    continue;
                }
                auto &recording = m_k4a_recordings[camera_index];
                if(!advance_slave_past_master(&recording)){
                    return false;
                }

                time_diff[camera_index] = static_cast<int64_t>(recording.current_ts) - static_cast<int64_t>(current_master_ts);

                // if time difference is bigger then 2ms then the frames are not sync and master has to be advanced again
                // for some reason timestamps can be a milli second before master?
                if(std::abs(time_diff[camera_index]) > max_diff_tolerance) {
                    spdlog::debug("camera {0} too far ahead of master, advance master, diff: {1}",camera_index, time_diff[camera_index]);
                    validImages = false;
                }
            }

            last_master_ts_ = current_master_ts;


        } while(!validImages);




        for(int cameraIndex=0;cameraIndex<m_k4a_recordings.size();++cameraIndex){
            if(m_k4a_recordings[cameraIndex].is_master){
                //spdlog::debug( "master, ID : {0} advance frames: {1} time diff {3}",cameraIndex, advanced_frames_[cameraIndex],time_diff[cameraIndex]);
            } else {
                //spdlog::debug( "subord, ID : {0} advance frames: {1} time diff {3}",cameraIndex, advanced_frames_[cameraIndex],time_diff[cameraIndex]);
            }

        }

        return !reachedEnd;
    }

    std::vector<cv::Mat > MultiKinectPlayback::getRawImages() {
        return m_currentRawImages;

    }

    std::vector<cv::Mat > MultiKinectPlayback::getUndistoredImages() {
        return m_currentUnsidtoredImages;
    }

    traact::vision::CameraCalibration MultiKinectPlayback::get_K4A_Intrinsics(recording_t *recording, int idx) {
        k4a_calibration_t k4acalib;

        k4a_result_t functionresult = k4a_playback_get_calibration(recording->handle, &k4acalib);

        if (functionresult == K4A_RESULT_FAILED)
            throw std::runtime_error("k4a record error: could not read calibration");

        k4a_calibration_camera_t sensor_calib;
        switch (idx) {
            case 0:
            case 2:
                sensor_calib = k4acalib.depth_camera_calibration;
                break;
            case 1:
                sensor_calib = k4acalib.color_camera_calibration;
                break;
            default:
                std::runtime_error("unknownn id for k4a calibration");

        }


        traact::vision::CameraCalibration result;


        if (functionresult == K4A_RESULT_FAILED)
            throw std::runtime_error("k4a record error: could not read calibration");

        k4a_calibration_camera_t params;
        params = k4acalib.depth_camera_calibration;

        result.fx = static_cast<float>(params.intrinsics.parameters.param.fx);
        result.fy = static_cast<float>(params.intrinsics.parameters.param.fy);
        result.cx = static_cast<float>(params.intrinsics.parameters.param.cx);
        result.cy = static_cast<float>(params.intrinsics.parameters.param.cy);
        result.skew = 0;

        result.radial_distortion.resize(6);
        result.tangential_distortion.resize(2);

        result.radial_distortion[0] = static_cast<float>(params.intrinsics.parameters.param.k1);
        result.radial_distortion[1] = static_cast<float>(params.intrinsics.parameters.param.k2);
        result.radial_distortion[2] = static_cast<float>(params.intrinsics.parameters.param.k3);
        result.radial_distortion[3] = static_cast<float>(params.intrinsics.parameters.param.k4);
        result.radial_distortion[4] = static_cast<float>(params.intrinsics.parameters.param.k5);
        result.radial_distortion[5] = static_cast<float>(params.intrinsics.parameters.param.k6);
        result.tangential_distortion[0] = static_cast<float>(params.intrinsics.parameters.param.p1);
        result.tangential_distortion[1] = static_cast<float>(params.intrinsics.parameters.param.p2);

        result.width = params.resolution_width;
        result.height = params.resolution_height;


        return result;


        return result;

    }


    bool MultiKinectPlayback::advanceFrameForCamera(recording_t *recording) {

        k4a_image_t image = NULL;
        do {
            k4a_stream_result_t stream_result = k4a_playback_get_next_capture(recording->handle, &recording->capture);
            if (stream_result != K4A_STREAM_RESULT_SUCCEEDED) {
                spdlog::warn( " error trying to advance frame, maybe end of file");
                return false;
            }
            image = k4a_capture_get_ir_image(recording->capture);

        } while (image == NULL);


        recording->current_ts = k4a_image_get_device_timestamp_usec(image);

        //k4a_image_release(image);
        cv::Mat img;
        u_int8_t *bufferp = k4a_image_get_buffer(image);
        size_t bufferSize = k4a_image_get_size(image);
        int stride = k4a_image_get_stride_bytes(image);
        int w = k4a_image_get_width_pixels(image);
        int h = k4a_image_get_height_pixels(image);
        img = cv::Mat(cv::Size(w, h), CV_16UC1, static_cast<void *>(bufferp), static_cast<size_t>(stride));

        cv::Mat tmp;
        img.convertTo(tmp, CV_MAKETYPE(CV_MAT_DEPTH(CV_8UC1), 1), 255. / m_irToGrayFactor);
        cv::Mat tmp_undis;

        int camera_index = recording->camera_index;
        m_undistorter[camera_index]->UndistortImage(tmp, tmp_undis);


        m_currentRawImages[camera_index] = tmp;
        m_currentUnsidtoredImages[camera_index] = tmp_undis;


        k4a_image_release(image);
        k4a_capture_release(recording->capture);

        return true;

    }

    void MultiKinectPlayback::process_image(int camera_index) {
        auto &recording = m_k4a_recordings[camera_index];


        k4a_image_t image = k4a_capture_get_ir_image(recording.capture);

        cv::Mat img;
        u_int8_t *bufferp = k4a_image_get_buffer(image);
        size_t bufferSize = k4a_image_get_size(image);
        int stride = k4a_image_get_stride_bytes(image);
        int w = k4a_image_get_width_pixels(image);
        int h = k4a_image_get_height_pixels(image);
        img = cv::Mat(cv::Size(w, h), CV_16UC1, static_cast<void *>(bufferp), static_cast<size_t>(stride));

        cv::Mat tmp;
        img.convertTo(tmp, CV_MAKETYPE(CV_MAT_DEPTH(CV_8UC1), 1), 255. / m_irToGrayFactor);
        cv::Mat tmp_undis;

        m_undistorter[camera_index]->UndistortImage(tmp, tmp_undis);


        m_currentRawImages[camera_index] = tmp;
        m_currentUnsidtoredImages[camera_index] = tmp_undis;


        k4a_image_release(image);
        k4a_capture_release(recording.capture);
    }

    bool MultiKinectPlayback::advance_master() {
        auto &master_recording = m_k4a_recordings[master_index];
        bool master_valid = false;

        uint64_t current_master_ts = 0;
        while (!master_valid) {
            if (!advanceFrameForCamera(&master_recording))
                return false;
            current_master_ts = master_recording.current_ts;
            if (current_master_ts <= last_master_ts_) {
                spdlog::error("frame has same timestamp as previous, skip");
            } else {
                master_valid = true;
            }
        }
        return master_valid;
    }

    bool MultiKinectPlayback::advance_slave_past_master(recording_t *recording) {
        uint64_t image_ts = 0;
        bool image_valid = false;
        while (!image_valid) {

            image_ts = recording->current_ts;
            uint32_t offset = recording->record_config.subordinate_delay_off_master_usec;
            uint64_t current_master_ts = m_k4a_recordings[master_index].current_ts;
            int64_t diff = static_cast<int64_t>(image_ts) - static_cast<int64_t>(current_master_ts);
            // image should have delay as offset between master and subordinate
            // because it does not always work like that, these fixes:
            // if timestamp is within 2ms (for some reason I had -1ms difference in one video)
            // image is after master, broken frames lead to skipped frames, advance master again
            //if (diff < offset || std::abs(diff) <= max_diff_tolerance || diff > 0) {
            if(std::abs(diff) <= max_diff_tolerance || diff > 0) {
                return true;
            }

            spdlog::debug("advance frame for camera {0}",recording->camera_index);
            //k4a_capture_release(recording->capture);
            if (!advanceFrameForCamera(recording)) {
                return false;
            }


        }
        return false;
    }

}
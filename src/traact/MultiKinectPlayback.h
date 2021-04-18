//
// Created by frieder on 02.02.20.
//

#ifndef PCPD_MULTIKINECTPLAYBACK_H
#define PCPD_MULTIKINECTPLAYBACK_H

#include <malloc.h>
#include <k4a/k4a.h>
#include <k4arecord/k4arecord_export.h>
#include <k4arecord/playback.h>
#include <k4arecord/record.h>
#include <boost/filesystem/path.hpp>
#include <traact/vision/UndistortionHelper.h>
#include <traact/vision_datatypes.h>
#include <traact/datatypes.h>
namespace traact::ba {


    typedef struct {
        k4a_playback_t handle;
        k4a_record_configuration_t record_config;
        k4a_capture_t capture;
        bool is_master;
        uint64_t current_ts;
        int camera_index;
    } recording_t;

    class MultiKinectPlayback {
    public:
        MultiKinectPlayback(std::string recordFolder, double irToGrayFactor, int camera_count);

        size_t getCameraCount();

        std::string getCameraFolder(size_t index);
        std::string getCameraFolderFileNAme(size_t index, std::string filename);

        bool advanceFrame();

        std::vector<cv::Mat > getRawImages();

        std::vector<cv::Mat > getUndistoredImages();



        traact::vision::CameraCalibration GetUndistortedCalibrfation(int index) {
            return m_undistorter[index]->GetUndistortedCalibration();
        }

        boost::filesystem::path GetCameraPath(int index) {
            return m_cameraFolder[index];
        }

    protected:
        int master_index;
        std::vector<boost::filesystem::path> m_cameraFolder;
        std::vector<recording_t> m_k4a_recordings;
        std::vector<std::shared_ptr<traact::vision::UndistortionHelper> > m_undistorter;

        std::vector<cv::Mat > m_currentRawImages;
        std::vector<cv::Mat> m_currentUnsidtoredImages;
        uint64_t last_master_ts_;
        double m_irToGrayFactor;
        std::vector<int> advanced_frames_;
        int64_t max_diff_tolerance{13000};

        traact::vision::CameraCalibration get_K4A_Intrinsics(recording_t *recording, int idx);

        traact::vision::CameraCalibration get_K4A_ColorIntrinsics( recording_t* recording ) {
            return get_K4A_Intrinsics(recording, 1);
        }

        traact::vision::CameraCalibration get_K4A_IRIntrinsics( recording_t* recording ) {
            return get_K4A_Intrinsics(recording, 0);
        }



        bool advanceFrameForCamera(recording_t* recording);
        void process_image(int camera_index);
        bool advance_master();
        bool advance_slave_past_master(recording_t* recording);

    };

}

#endif //PCPD_MULTIKINECTPLAYBACK_H

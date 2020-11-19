#include<iostream>
#include<opencv2/aruco.hpp>
#include<opencv2/opencv.hpp>
#include<librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>
using namespace std;

constexpr size_t WIDTH = 640;
constexpr size_t HEIGHT = 360;
constexpr double PAUL_L = 100;
constexpr double PAUL_R = 540;
constexpr double AREA_H = 100;
constexpr double AREA_L = 200;
constexpr double DEPTH_MAX = 3;
constexpr double DEPTH_MIN = 2;

int main(int argc, char **argv) try {
    rs2::colorizer color_map;
    rs2::config cfg;
    cfg.enable_stream(RS2_STREAM_COLOR, WIDTH, HEIGHT, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_DEPTH, WIDTH, HEIGHT, RS2_FORMAT_Z16, 30);
    rs2::pipeline pipe;
    auto profile = pipe.start(cfg);
    auto depth_stream = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    auto intr = depth_stream.get_intrinsics();

    //dictionary
    const cv::aruco::PREDEFINED_DICTIONARY_NAME dictionary_name = cv::aruco::DICT_4X4_50;
    cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(dictionary_name);

    vector<int> marker_ids;
    vector<std::vector<cv::Point2f>> marker_corners;
    cv::Ptr<cv::aruco::DetectorParameters> parameters = cv::aruco::DetectorParameters::create();

    double marker_x, marker_y;

    while(1) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::align align_to_color(RS2_STREAM_COLOR);
        auto aligned_frames = align_to_color.process(frames);
        auto depth_map = aligned_frames.get_depth_frame();
        auto color_map = aligned_frames.get_color_frame();
        rs2::depth_frame dep = aligned_frames.get_depth_frame();
        cv::Mat image (cv::Size(color_map.get_width(), color_map.get_height()), CV_8UC3, (void *)color_map.get_data(), cv::Mat::AUTO_STEP);

        if(cv::waitKey(1) == 'q') {
            cout << "finish!!" << endl;
            break;
        }
    }
}

catch (const rs2::error &e) {
    cerr << "Realsense error calling" << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception &e) {
    cerr << e.what() << endl;
    return EXIT_FAILURE;
}

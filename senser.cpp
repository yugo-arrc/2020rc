#include<iostream>
#include<opencv2/opencv.hpp>
#include<opencv2/aruco.hpp>
#include<librealsense2/rs.hpp>
#include<librealsense2/rsutil.h>
//using namespace std;

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

    while(1) {
        rs2::frameset frames = pipe.wait_for_frames();
        rs2::align align_to_color(RS2_STREAM_COLOR);
        auto aligned_frames = align_to_color.process(frames);
        auto depth_map = aligned_frames.get_depth_frame();
        auto color_map = aligned_frames.get_color_frame();
        rs2::depth_frame dep = aligned_frames.get_depth_frame();
        cv::Mat image(cv::Size(color_map.get_width(), color_map.get_height()), CV_8UC3, (void *)color_map.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat mono, gray, image_ed;

        double depth;
        for(int cell_x = PAUL_L; cell_x <= PAUL_R; cell_x += 2) {
            for(int cell_y = AREA_H; cell_y <= AREA_L; cell_y += 2) {
                double depth = dep.get_distance(cell_x, cell_y);
                if(depth < DEPTH_MAX && depth > DEPTH_MIN) {
                    cv::circle(image, cv::Point(cell_x, cell_y), 1, cv::Scalar(255, 255, 255), -1);
                }
            }
        }
        cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        cv::threshold(gray, mono, 254, 255, cv::THRESH_BINARY);

        dilate(mono, image_ed, cv::Mat(), cv::Point(-1, -1), 1);
        erode(image_ed, image_ed, cv::Mat(), cv::Point(-1, -1), 1);
        erode(image_ed, image_ed, cv::Mat(), cv::Point(-1, -1), 1);
        dilate(image_ed, image_ed, cv::Mat(), cv::Point(-1, -1), 1);

        cv::imshow("www", image_ed);
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

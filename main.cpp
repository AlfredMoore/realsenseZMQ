/**
 * This file is part of Photo-SLAM.
 *
 * This is the COMPLETE client part of the RealSense RGB-D example using ZeroMQ.
 * It reads RGB and Depth frames from a RealSense camera, performs detailed
 * sensor configuration, and publishes the frames and timestamps
 * via a ZeroMQ socket.
 */

#include <iostream>
#include <vector>
#include <string>
#include <atomic>
#include <csignal>
#include <algorithm>

#include <librealsense2/rs.hpp>
#include <librealsense2/rsutil.h>
#include <opencv2/opencv.hpp>
#include <opencv2/core/core.hpp>
#include <zmq.hpp>
#include <chrono>
#include <argparse/argparse.hpp>

std::atomic<bool> terminate_program(false);

void signal_handler(int signal_num) {
    if (signal_num == SIGINT) {
        terminate_program = true;
        std::cout << "SIGINT received. Terminating client..." << std::endl;
    }
}

// Helper function from realsense_rgbd.cpp to print sensor options
static void get_sensor_option(const rs2::sensor& sensor)
{
    std::cout << "Sensor supports the following options:\n" << std::endl;
    for (int i = 0; i < static_cast<int>(RS2_OPTION_COUNT); i++)
    {
        rs2_option option_type = static_cast<rs2_option>(i);
        if (sensor.supports(option_type))
        {
            std::cout << "  " << i << ": " << option_type
                      << "\n    Description   : " << sensor.get_option_description(option_type)
                      << "\n    Current Value : " << sensor.get_option(option_type) << std::endl;
        }
        else
        {
            std::cout << "  " << i << ": " << option_type << " is not supported" << std::endl;
        }
    }
}

int main(int argc, char** argv) {
    std::signal(SIGINT, signal_handler);
    // --- argparse ---
    argparse::ArgumentParser program("REALSENSE_ZMQ_CLIENT");
    program.add_argument("--serial")
        .help("Specify the serial number of the RealSense device")
        .default_value("000000000000");

    program.add_argument("--show")
        .help("Show images for debugging")
        .default_value(false)
        .implicit_value(true);

    try {
        program.parse_args(argc, argv);
    } catch (const std::runtime_error& err) {
        std::cerr << "Error: " << err.what() << std::endl;
        return 1;
    }

    std::string desired_serial = program.get<std::string>("--serial");
    std::cout << "Using RealSense device with serial: " << desired_serial << std::endl;

    bool show_images = program.get<bool>("--show");
    std::cout << "Show images: " << show_images << std::endl;

    // --- ZeroMQ Publisher Setup ---
    zmq::context_t context(1);
    zmq::socket_t socket(context, ZMQ_PUB);
    try {
        socket.bind("tcp://*:5555");       // TODO: no hardcoding
        std::cout << "ZMQ Publisher listening on tcp://*:5555" << std::endl;
    } catch (const zmq::error_t& e) {
        std::cerr << "ZMQ Error: Could not bind socket. " << e.what() << std::endl;
        return 1;
    }

    // --- RealSense Detailed Setup (from realsense_rgbd.cpp) ---
    rs2::context ctx;
    rs2::device_list devices = ctx.query_devices();
    rs2::device selected_device;
    bool device_found = false;

    if (devices.size() == 0) {
        std::cerr << "No device connected, please connect a RealSense device." << std::endl;
        return 1;
    }

    std::cout << "##########\nPlease record serial number of your desired device." << std::endl;
    for (rs2::device device : devices) {
        std::cout << "Found RealSense device:" 
                  << device.get_info(RS2_CAMERA_INFO_NAME) 
                  << " | Serial: " << device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
                  << " | USB Port: " << device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT)
                  << std::endl;
        if (std::string(device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)) == desired_serial) {
            selected_device = device;
            device_found = true;
        }
    }
    std::cout << "##########" << std::endl;

    if (!device_found) {
        std::cerr << "Desired RealSense device not found. Please check the serial and USB port." << std::endl;
        return 1;
    } else {
        std::cout << "Desired RealSense device found!" << std::endl;
        std::cout << "Name: " << selected_device.get_info(RS2_CAMERA_INFO_NAME) << std::endl;
        std::cout << "Serial: " << selected_device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) << std::endl;
        std::cout << "USB Port: " << selected_device.get_info(RS2_CAMERA_INFO_PHYSICAL_PORT) << std::endl;
    }
    
    // --- Configure sensors ---
    std::vector<rs2::sensor> sensors = selected_device.query_sensors();

    int index = 0;
    // We can now iterate the sensors and print their names
    for (rs2::sensor sensor : sensors){
        if (sensor.supports(RS2_CAMERA_INFO_NAME))
        {
            ++index;
            if (index == 1) // irSensor
            {
                sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                sensor.set_option(RS2_OPTION_EMITTER_ENABLED, 1); // emitter on for depth information
            }
            get_sensor_option(sensor);
            if (index == 2) // RGB camera
            {
                // Fixed Exposure
                // sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0);
                sensor.set_option(RS2_OPTION_EXPOSURE, 150);
                sensor.set_option(RS2_OPTION_GAIN, 64);

                // // Auto Exposure
                // sensor.set_option(RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1);
                // // sensor.set_option(RS2_OPTION_EXPOSURE, 150);
                // // sensor.set_option(RS2_OPTION_GAIN, 64);
            }

            if (index == 3)
                sensor.set_option(RS2_OPTION_ENABLE_MOTION_CORRECTION, 0);
        }
    }

    rs2::pipeline pipe;
    rs2::config cfg;
    // Note: To add IMU, you would enable these streams and send the data.
    // This requires server-side changes to parse and use the IMU data.
    // cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    // cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_device(desired_serial);
    cfg.enable_stream(RS2_STREAM_DEPTH, 1280, 720, RS2_FORMAT_Z16, 30);
    cfg.enable_stream(RS2_STREAM_COLOR, 1280, 720, RS2_FORMAT_RGB8, 30);    // RGB stream
    
    rs2::pipeline_profile profile = pipe.start(cfg);
    rs2::align align_to_color(RS2_STREAM_COLOR);

    std::cout << "RealSense camera started and streams are configured." << std::endl;
    if (!show_images) {
        std::cout << "Not showing images" << std::endl;
    }

    while (!terminate_program) {
        rs2::frameset frames = pipe.wait_for_frames();
        if (!frames) continue;

        auto aligned_frames = align_to_color.process(frames);

        rs2::video_frame color_frame = aligned_frames.get_color_frame();
        rs2::depth_frame depth_frame = aligned_frames.get_depth_frame();
        
        if (!color_frame || !depth_frame) continue;

        double timestamp = color_frame.get_timestamp(); // Use frame timestamp

        // --- Create OpenCV Mats ---
        cv::Mat color_rgb(cv::Size(1280, 720), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
        cv::Mat depth(cv::Size(1280, 720), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

        // --- Encode and Send Images ---
        std::vector<uchar> rgb_buffer;
        cv::imencode(".jpg", color_rgb, rgb_buffer, {cv::IMWRITE_JPEG_QUALITY, 90});
        
        std::vector<uchar> depth_buffer;
        cv::imencode(".png", depth, depth_buffer, {cv::IMWRITE_PNG_COMPRESSION, 1});
        //cv::imencode(".png", depth, depth_buffer); // PNG for lossless 16-bit depth

        std::string timestamp_str = std::to_string(timestamp);

        // Send as a multi-part message: [RGB, Depth, Timestamp]
        socket.send(zmq::buffer(rgb_buffer), zmq::send_flags::sndmore);
        socket.send(zmq::buffer(depth_buffer), zmq::send_flags::sndmore);
        socket.send(zmq::buffer(timestamp_str), zmq::send_flags::none);

        // Optional: display images on client side for verification
        if (show_images) {
            cv::Mat color_bgr;
            cv::cvtColor(color_rgb, color_bgr, cv::COLOR_RGB2BGR);
            cv::imshow("Client - RGB (Converted to BGR for Display)", color_bgr);
            // cv::imshow("Client - Depth", depth * 15); // Scale for visualization
            if (cv::waitKey(1) == 27) { // ESC key to exit
                terminate_program = true;
            }
        }

    }

    std::cout << "Client shutting down..." << std::endl;
    socket.close();
    context.close();
    cv::destroyAllWindows();
    
    return 0;
}

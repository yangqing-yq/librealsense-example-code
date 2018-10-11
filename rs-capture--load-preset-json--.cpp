// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering

//qing
#include <librealsense2/rs_advanced_mode.hpp>
#include <fstream>


// Capture Example demonstrates how to
// capture depth and color video streams and render them to the screen
int main(int argc, char * argv[]) try
{
    rs2::log_to_console(RS2_LOG_SEVERITY_ERROR);
    // Create a simple OpenGL window for rendering:
    window app(1280, 720, "RealSense Capture Example");
    // Declare two textures on the GPU, one for color and one for depth
    texture depth_image, color_image;

    // Declare depth colorizer for pretty visualization of depth data
    rs2::colorizer color_map;


	//qing
	rs2::context ctx;
	rs2::pipeline pipe(ctx);
	auto devices = ctx.query_devices();
	size_t device_count = devices.size();
	if (!device_count)
	{
		std::cout << "No device detected. Is it plugged in?\n";
		return EXIT_SUCCESS;
	}
	auto dev = devices[0];
	//read json file
	std::ifstream file("C:\\2AndersShortNew.json");
	std::string json_content((std::istreambuf_iterator<char>(file)), std::istreambuf_iterator<char>());
	//enable advance mode & load json
	if (dev.is<rs400::advanced_mode>())
	{
		auto advanced_mode_dev = dev.as<rs400::advanced_mode>();
		if (!advanced_mode_dev.is_enabled())
			advanced_mode_dev.toggle_advanced_mode(true);
		advanced_mode_dev.load_json(json_content);
	}

    // Start streaming with default recommended configuration
    pipe.start();

    while(app) // Application still alive?
    {
        rs2::frameset data = pipe.wait_for_frames(); // Wait for next set of frames from the camera

        rs2::frame depth = color_map.process(data.get_depth_frame()); // Find and colorize the depth data
        rs2::frame color = data.get_color_frame();            // Find the color data

        // For cameras that don't have RGB sensor, we'll render infrared frames instead of color
        if (!color)
            color = data.get_infrared_frame();

        // Render depth on to the first half of the screen and color on to the second
        depth_image.render(depth, { 0,               0, app.width() / 2, app.height() });
        color_image.render(color, { app.width() / 2, 0, app.width() / 2, app.height() });
    }

    return EXIT_SUCCESS;
}
catch (const rs2::error & e)
{
    std::cerr << "RealSense error calling " << e.get_failed_function() << "(" << e.get_failed_args() << "):\n    " << e.what() << std::endl;
    return EXIT_FAILURE;
}
catch (const std::exception& e)
{
    std::cerr << e.what() << std::endl;
    return EXIT_FAILURE;
}

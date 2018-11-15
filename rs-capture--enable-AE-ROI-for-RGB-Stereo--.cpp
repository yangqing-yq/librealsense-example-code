// License: Apache 2.0. See LICENSE file in root directory.
// Copyright(c) 2017 Intel Corporation. All Rights Reserved.

#include <librealsense2/rs.hpp> // Include RealSense Cross Platform API
#include "example.hpp"          // Include short list of convenience functions for rendering
#include <thread>
//---add windows.h---
#include "windows.h"



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

	// Declare RealSense pipeline, encapsulating the actual device and sensors
	rs2::context ctx;
	rs2::pipeline pipe(ctx);
	// Start streaming with default recommended configuration
	pipe.start();

	//------------------------Start of set AE roi for RGB and Stereo----------------------
	//set roi value for x,y axis in pixel map. Below is just an example for 10% center ROI.
	int xs = 640 * 0.45 , xe = 640 * 0.55, ys = 480 * 0.45, ye = 480 * 0.55;
	rs2::region_of_interest roi{};
	roi.min_x = static_cast<int>(xs);
	roi.max_x = static_cast<int>(xe);
	roi.min_y = static_cast<int>(ys);
	roi.max_y = static_cast<int>(ye);
	//init roi2 to check result
	auto sensors = ctx.query_all_sensors();
	for (auto sensor = sensors.rbegin(); sensor != sensors.rend(); ++sensor) {

		//// if depth need AE ROI feature enabled, uncomment below block of code
		//if (sensor->is<rs2::depth_stereo_sensor>()) {
		//	std::cout << "stereo sensor" << std::endl;
		//	continue;
		//}

		std::this_thread::sleep_for(std::chrono::milliseconds(200));

		if (sensor->is<rs2::roi_sensor>()) {
			try {
				//set AE roi to sensor firmware
				sensor->as<rs2::roi_sensor>().set_region_of_interest(roi);
				//get roi info from sensor to check if set AE roi succeed
			}
			catch (std::exception& e) {
				std::cerr << e.what() << std::endl;
			}
		}

	}
	//-------------------------End of set AE roi for RGB and Stereo-----------------------


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

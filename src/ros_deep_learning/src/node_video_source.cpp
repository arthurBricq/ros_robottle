/*
 * Copyright (c) 2020, NVIDIA CORPORATION. All rights reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include "ros_compat.h"
#include "image_converter.h"

#include <jetson-utils/videoSource.h>

#include "std_msgs/msg/string.hpp"


// globals	
videoSource* stream = NULL;
imageConverter* image_cvt = NULL;
Publisher<sensor_msgs::Image> image_pub = NULL;

videoOptions video_options;
std::string resource_str;

// aquire and publish camera frame
bool aquireFrame()
{
	imageConverter::PixelType* nextFrame = NULL;

	// get the latest frame
	if( !stream->Capture(&nextFrame, 1000) )
	{
		ROS_ERROR("failed to capture next frame");
		return false;
	}

	// assure correct image size
	if( !image_cvt->Resize(stream->GetWidth(), stream->GetHeight(), imageConverter::ROSOutputFormat) )
	{
		ROS_ERROR("failed to resize camera image converter");
		return false;
	}

	// populate the message
	sensor_msgs::Image msg;

	if( !image_cvt->Convert(msg, imageConverter::ROSOutputFormat, nextFrame) )
	{
		ROS_ERROR("failed to convert video stream frame to sensor_msgs::Image");
		return false;
	}

	// populate timestamp in header field
	msg.header.stamp = ROS_TIME_NOW();

	// publish the message
	image_pub->publish(msg);
	ROS_DEBUG("published %ux%u video frame", stream->GetWidth(), stream->GetHeight());
	
	return true;
}

// Callback for camera control topic
void flip_topic_callback(const std_msgs::msg::String::SharedPtr input) {
    ROS_ERROR("Message Receveid");

    std::string s = input->data.c_str();
    ROS_ERROR("flipping camera");
    if (s == "normal"){
        video_options.flipMethod = videoOptions::FlipMethodFromStr("rotate-180");
    }
    else if (s == "flip"){
        video_options.flipMethod = videoOptions::FlipMethodFromStr("counterclockwise");
    }
    // delete the stream and create a new one imediatly
	delete stream;
	stream = videoSource::Create(resource_str.c_str(), video_options);
}


void timer_callback() {
    if( !aquireFrame() )
    {
        if( !stream->IsStreaming() )
        {
            ROS_INFO("stream is closed or reached EOS, exiting node...");
        }
    }
}

// node main loop
int main(int argc, char **argv)
{
	/*
	 * create node instance
	 */
	ROS_CREATE_NODE("video_source");

	/*
	 * declare parameters
	 */

	std::string codec_str;

	int video_width = video_options.width;
	int video_height = video_options.height;

	ROS_DECLARE_PARAMETER("resource", resource_str);
	ROS_DECLARE_PARAMETER("codec", codec_str);
	ROS_DECLARE_PARAMETER("width", video_width);
	ROS_DECLARE_PARAMETER("height", video_height);
	ROS_DECLARE_PARAMETER("framerate", video_options.frameRate);
	ROS_DECLARE_PARAMETER("loop", video_options.loop);
	
	/*
	 * retrieve parameters
	 */
	ROS_GET_PARAMETER("resource", resource_str);
	ROS_GET_PARAMETER("codec", codec_str);
	ROS_GET_PARAMETER("width", video_width);
	ROS_GET_PARAMETER("height", video_height);
	ROS_GET_PARAMETER("framerate", video_options.frameRate);
	ROS_GET_PARAMETER("loop", video_options.loop);
	
	if( resource_str.size() == 0 )
	{
		ROS_ERROR("resource param wasn't set - please set the node's resource parameter to the input device/filename/URL");
		return 0;
	}

	if( codec_str.size() != 0 )
		video_options.codec = videoOptions::CodecFromStr(codec_str.c_str());

	video_options.width = video_width;
	video_options.height = video_height;
    // this line to rotate the camera of 180 degrees
    video_options.flipMethod = videoOptions::FlipMethodFromStr("rotate-180");

	ROS_INFO("opening video source: %s", resource_str.c_str());

	/*
	 * open video source
	 */
	stream = videoSource::Create(resource_str.c_str(), video_options);

	if( !stream )
	{
		ROS_ERROR("failed to open video source");
		return 0;
	}


	/*
	 * create image converter
	 */
	image_cvt = new imageConverter();

	if( !image_cvt )
	{
		ROS_ERROR("failed to create imageConverter");
		return 0;
	}


	/*
	 * advertise publisher topics
	 */
	ROS_CREATE_PUBLISHER(sensor_msgs::Image, "raw", 2, image_pub);


    /*
     * subscribe to topic to flip the camera
     */
    
    auto flip_topic = ROS_CREATE_SUBSCRIBER(std_msgs::msg::String, "flip_topic", 5, flip_topic_callback);

	/*
	 * start the camera streaming
	 */
	if( !stream->Open() )
	{
		ROS_ERROR("failed to start streaming video source");
		return 0;
	}


	/*
	 * start publishing video frames
	 */

    auto timer_ = node->create_wall_timer(std::chrono::milliseconds(50), [&](){timer_callback();});
	ROS_SPIN();

	/*
	 * free resources
	 */
	delete stream;
	delete image_cvt;

	return 0;
}


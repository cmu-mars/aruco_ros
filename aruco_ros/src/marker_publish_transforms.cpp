/**
* @file marker_publish_transforms.cpp
* @author Bence Magyar
* @date June 2012
* @version 0.1
* @brief publish transforms associated with all visible markers
*/

#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>

#include <opencv2/core/core.hpp>
#include <ros/ros.h>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <aruco_ros/aruco_ros_utils.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_datatypes.h>

#include <dynamic_reconfigure/server.h>
#include <aruco_ros/ArucoThresholdConfig.h>
#include <string>

using namespace cv;
using namespace aruco;
using namespace std;

cv::Mat inImage;
aruco::CameraParameters camParam;
bool useRectifiedImages, normalizeImageIllumination;
int dctComponentsToRemove;
MarkerDetector mDetector;
vector<Marker> markers;
ros::Subscriber cam_info_sub;
bool cam_info_received;
image_transport::Publisher image_pub;


std::string parent_name;
double marker_size;

void image_callback(const sensor_msgs::ImageConstPtr& msg)
{
	double ticksBefore = cv::getTickCount();
	static tf::TransformBroadcaster br;
	if (cam_info_received)
	{
		ros::Time curr_stamp(ros::Time::now());
		cv_bridge::CvImagePtr cv_ptr;
		try
		{
			cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::RGB8);
			inImage = cv_ptr->image;

			markers.clear();
			// Detect the markers
			mDetector.detect(inImage, markers, camParam, marker_size);
			// For each marker, draw info and its boundaries in the image
			for (unsigned int i=0; i<markers.size(); ++i) {
				tf::Transform transform = aruco_ros::arucoMarker2Tf(markers[i]);
				std::stringstream ss;
				ss << markers[i].id;;

				std::string mid = "Marker" + ss.str();

				br.sendTransform(tf::StampedTransform(transform, curr_stamp, parent_name, mid.c_str()));
				
				markers[i].draw(inImage, Scalar(0,0,255), 2);
			}

			cv::circle(inImage, cv::Point(inImage.cols/2, inImage.rows/2), 4, cv::Scalar(0,255,0), 1);

			if (camParam.isValid() && marker_size!=-1) {
				for (unsigned int i = 0; i<markers.size(); ++i) {
					CvDrawingUtils::draw3dCube(inImage, markers[i], camParam);
				}
			}

			if (image_pub.getNumSubscribers() > 0) {
				// show input with augmented information
				cv_bridge::CvImage out_msg;
				out_msg.header.stamp = curr_stamp;
				out_msg.encoding = sensor_msgs::image_encodings::RGB8;
				out_msg.image = inImage;
				image_pub.publish(out_msg.toImageMsg());

			}


		}
		catch (cv_bridge::Exception& e) {
			ROS_ERROR("cv_bridge exception: %s", e.what());
			return;
		}
	}
}

void cam_info_callback(const sensor_msgs::CameraInfo &msg) {
	camParam = aruco_ros::rosCameraInfo2ArucoCamParams(msg, useRectifiedImages);
	cam_info_received = true;
	cam_info_sub.shutdown();
}

void reconf_callback(aruco_ros::ArucoThresholdConfig& config, uint32_t level) {
	mDetector.setThresholdParams(config.param1,config.param2);
	normalizeImageIllumination = config.normalizeImage;
	dctComponentsToRemove = config.dctComponentsToRemove;
}

int main(int argc, char** argv) {
	ros::init(argc, argv, "aruco_marker_publish_transforms");
	ros::NodeHandle nh("~");
	image_transport::ImageTransport it(nh);

	dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig> server;
	dynamic_reconfigure::Server<aruco_ros::ArucoThresholdConfig>::CallbackType f_;
	f_ = boost::bind(&reconf_callback, _1, _2);
	server.setCallback(f_);

	normalizeImageIllumination = false;

	nh.param<bool>("image_is_rectified", useRectifiedImages, true);
	ROS_INFO_STREAM("Image is rectified: " << useRectifiedImages);

	image_transport::Subscriber image_sub = it.subscribe("/image", 1, &image_callback);
	cam_info_sub = nh.subscribe("/camera_info", 1, &cam_info_callback);

	cam_info_received = false;
	image_pub = it.advertise("result", 1);

	nh.param<double>("marker_size", marker_size, 0.05);
	nh.param<bool>("normalizeImage", normalizeImageIllumination, true);
	nh.param<int>("dct_components_to_remove", dctComponentsToRemove, 2);
	if (dctComponentsToRemove == 0)
		normalizeImageIllumination = false;
	nh.param<std::string>("parent_name", parent_name, "");

	if (parent_name=="") {
		ROS_ERROR("parent_name was not set!");
		return -1;
	}

	ROS_INFO("Aruco node started with marker size of %f meters", marker_size);
	ROS_INFO("Aruco node will publish pose to TF with (%s,marker<id>) as (parent,child).", parent_name.c_str());
	ros::spin();
}
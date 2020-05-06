/*
 * Copyright 2015-2019 Autoware Foundation. All rights reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <include/points_image/points_image.hpp>

#define CAMERAEXTRINSICMAT "CameraExtrinsicMat"
#define CAMERAMAT "CameraMat"
#define DISTCOEFF "DistCoeff"
#define IMAGESIZE "ImageSize"

#define __APP_NAME__ "points2image_node"

static cv::Mat cameraExtrinsicMat;
static cv::Mat cameraMat;
static cv::Mat distCoeff;
static cv::Size imageSize;

static ros::Publisher pub;


//载入相机内参和外参
static bool load_camera_mat(const std::string& calibration_file)
{
	if (calibration_file.empty())
	{
		ROS_ERROR("[%s] Missing calibration file path '%s'.", __APP_NAME__, calibration_file.c_str());
		ros::shutdown();
		return false;
	}

	cv::FileStorage fs(calibration_file, cv::FileStorage::READ);
	if (!fs.isOpened())
	{
		ROS_ERROR("[%s] Cannot open file calibration file '%s'", __APP_NAME__, calibration_file.c_str());
		ros::shutdown();
		return false;
	}

	fs["CameraExtrinsicMat"] >> cameraExtrinsicMat;
	fs["CameraMat"] >> cameraMat;
	fs["DistCoeff"] >> distCoeff;
	fs["ImageSize"] >> imageSize;
	//fs["DistModel"] >> DistModel;
	
	resetMatrix();
	return true;
}

static void callback(const sensor_msgs::PointCloud2ConstPtr& msg)
{
  if (cameraExtrinsicMat.empty() || cameraMat.empty() || distCoeff.empty() || imageSize.height == 0 ||
      imageSize.width == 0)
  {
    ROS_INFO("[points2image]Looks like camera_info or projection_matrix are not being published.. Please check that "
             "both are running..");
    return;
  }
  
  cv::Mat image(imageSize, CV_8UC3, cv::Scalar(255, 255, 255));
  
  pointcloud2_to_image(msg, image, cameraExtrinsicMat, cameraMat, distCoeff, imageSize);
  
  cv::imshow("dst",image);
  cv::waitKey(1);
  
  //发布图像
  //pub.publish
  
}

int main(int argc, char* argv[])
{
  ros::init(argc, argv, __APP_NAME__);
  ros::NodeHandle n;

  ros::NodeHandle private_nh("~");

  std::string calibration_file_name;
  std::string pub_topic_str = "/points_image";

  private_nh.param<std::string>("calibration_file", calibration_file_name, "");
  std::string points_topic = private_nh.param<std::string>("points_topic","/points_raw");
  
  if(calibration_file_name.empty())
  {
  	ROS_ERROR("[points2image] please set camera calibration file name!");
  	return 0;
  }
  
  if(!load_camera_mat(calibration_file_name))
	  return 0;

  std::string name_space_str = ros::this_node::getNamespace();

  if (name_space_str != "/")
  {
    if (name_space_str.substr(0, 2) == "//")
    {
      /* if name space obtained by ros::this::node::getNamespace()
         starts with "//", delete one of them */
      name_space_str.erase(name_space_str.begin());
    }
    pub_topic_str = name_space_str + pub_topic_str;
  }

  

  ROS_INFO("[points2image]Publishing to... %s", pub_topic_str.c_str());
  
  pub = n.advertise<sensor_msgs::Image>(pub_topic_str, 1);

  ros::Subscriber sub = n.subscribe(points_topic, 1, callback);

  ros::spin();
  return 0;
}

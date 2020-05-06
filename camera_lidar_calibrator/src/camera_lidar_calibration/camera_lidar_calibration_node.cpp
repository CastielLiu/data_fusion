/*
 * camera_lidar_calibration.cpp
 *
 *  Created on: Sep 4, 2017
 *      Author: amc-jp
 */

#include <string>
#include <vector>
#include <ctime>
#include <sstream>
#include <unistd.h>
#include <sys/types.h>
#include <pwd.h>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl_ros/point_cloud.h>
#include <boost/filesystem.hpp>

#include "opencv2/highgui/highgui.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#if (CV_MAJOR_VERSION == 3)
	#include <opencv2/imgcodecs.hpp>
#else
	#include <opencv2/contrib/contrib.hpp>
#endif

#define __APP_NAME__ "camera_lidar_calibration_node"

class ROSCameraLidarApp

{
	ros::NodeHandle 	node_handle_;
	ros::NodeHandle 	nh_private_;

	ros::Subscriber 	subscriber_image_raw_;
	ros::Subscriber 	subscriber_points_raw_;
	ros::Subscriber     subscriber_intrinsics_;
	ros::Subscriber     subscriber_clicked_point_;
	ros::Subscriber     subscriber_image_point_;

	cv::Size            image_size_;
	cv::Mat             camera_instrinsics_;
	cv::Mat             distortion_coefficients_;
	std::string         dist_model_;
	
	std::vector<cv::Point2f> clicked_image_points_;
	std::vector<cv::Point3f> clicked_velodyne_points_;
	
	bool is_rotation_matrix(const cv::Mat& in_matrix)
	{
		cv::Mat rotation_matrix;
		transpose(in_matrix, rotation_matrix);

		cv::Mat test_matrix = rotation_matrix * in_matrix;
		cv::Mat test_result = cv::Mat::eye(3,3, test_matrix.type());

		return cv::norm(test_result, test_matrix) < 1e-6;

	}

	cv::Point3f get_rpy_from_matrix(const cv::Mat& in_rotation)
	{

		assert(is_rotation_matrix(in_rotation));

		float sy = sqrt(in_rotation.at<double>(0,0) * in_rotation.at<double>(0,0) +
				                in_rotation.at<double>(1,0) * in_rotation.at<double>(1,0) );

		bool is_singular = sy < 1e-6;

		float x, y, z;
		if (!is_singular)
		{
			x = atan2(in_rotation.at<double>(2,1), in_rotation.at<double>(2,2));
			y = atan2(-in_rotation.at<double>(2,0), sy);
			z = atan2(in_rotation.at<double>(1,0), in_rotation.at<double>(0,0));
		}
		else
		{
			x = atan2(-in_rotation.at<double>(1,2), in_rotation.at<double>(1,1));
			y = atan2(-in_rotation.at<double>(2,0), sy);
			z = 0;
		}
		return cv::Point3f(RAD2DEG(x), RAD2DEG(y), RAD2DEG(z));
	}

	void SaveCalibrationFile(cv::Mat in_extrinsic)
	{
		
		std::string path_filename_str;
		const char *homedir;

		time_t rawtime;
		struct tm * timeinfo;
		char buffer[80];

		time (&rawtime);
		timeinfo = localtime(&rawtime);

		strftime(buffer,sizeof(buffer),"%Y%m%d_%H%M%S",timeinfo);
		std::string datetime_str(buffer);

		if ((homedir = getenv("HOME")) == NULL) {
			homedir = getpwuid(getuid())->pw_dir;
		}

		path_filename_str = std::string(homedir) + "/"+datetime_str+"_lidar_camera_calibration.yaml";

		cv::FileStorage fs(path_filename_str.c_str(), cv::FileStorage::WRITE);
		if(!fs.isOpened()){
			fprintf(stderr, "%s : cannot open file\n", path_filename_str.c_str());
			exit(EXIT_FAILURE);
		}

		fs << "CameraExtrinsicMat" << in_extrinsic;
		fs << "CameraMat" << camera_instrinsics_;
		fs << "DistCoeff" << distortion_coefficients_;
		fs << "ImageSize" << image_size_;
		fs << "ReprojectionError" << 0;
		fs << "DistModel" << dist_model_;

		ROS_INFO("Wrote Autoware Calibration file in: %s", path_filename_str.c_str());
	}

	void CalibrateSensors()
	{
		std::cout << "\nNumber of points [image:lidar] " <<clicked_image_points_.size() << ":"  << clicked_velodyne_points_.size() << std::endl;

		if (clicked_velodyne_points_.size() == clicked_image_points_.size()
		    && clicked_image_points_.size() > 8)
		{
			cv::Mat rotation_vector, translation_vector;

			cv::solvePnP(clicked_velodyne_points_,
			             clicked_image_points_,
			             camera_instrinsics_,
			             distortion_coefficients_,
			             rotation_vector,
			             translation_vector,
			             true,
			             CV_EPNP
			);

			cv::Mat rotation_matrix;
			cv::Rodrigues(rotation_vector, rotation_matrix);

			cv::Mat camera_velodyne_rotation = rotation_matrix.t();
			cv::Point3f camera_velodyne_point(translation_vector);
			cv::Point3f camera_velodyne_translation;

			camera_velodyne_translation.x = -camera_velodyne_point.z;
			camera_velodyne_translation.y = camera_velodyne_point.x;
			camera_velodyne_translation.z = camera_velodyne_point.y;

			std::cout << "Rotation:" << camera_velodyne_rotation << std::endl << std::endl;
			std::cout << "Translation:" << camera_velodyne_translation << std::endl;
			std::cout << "RPY: " << get_rpy_from_matrix(camera_velodyne_rotation) << std::endl << std::endl;

			cv::Mat extrinsics = cv::Mat::eye(4,4, CV_64F);
			camera_velodyne_rotation.copyTo(extrinsics(cv::Rect_<float>(0,0,3,3)));

			std::cout << extrinsics << std::endl;

			extrinsics.at<double>(0,3) = camera_velodyne_translation.x;
			extrinsics.at<double>(1,3) = camera_velodyne_translation.y;
			extrinsics.at<double>(2,3) = camera_velodyne_translation.z;

			std::cout << extrinsics << std::endl;

			SaveCalibrationFile(extrinsics);
		}
	}

	void RvizClickedPointCallback(const geometry_msgs::PointStamped& in_clicked_point)
	{
		//雷达点数已经等于图像点数，用当前雷达点覆盖上一雷达点
		//错选后可重新选择
		if(clicked_velodyne_points_.size() == clicked_image_points_.size() )
		{
			if(clicked_velodyne_points_.size()==0)
			{
				ROS_WARN("please click point in image firstly!");
				return;
			}
			clicked_velodyne_points_[clicked_velodyne_points_.size()-1] 
				= cv::Point3f(in_clicked_point.point.x, in_clicked_point.point.y, in_clicked_point.point.z);
		}
		else
			clicked_velodyne_points_.push_back(cv::Point3f(in_clicked_point.point.x,
														   in_clicked_point.point.y,
														   in_clicked_point.point.z));
		std::cout << cv::Point3f(in_clicked_point.point.x, in_clicked_point.point.y, in_clicked_point.point.z) << std::endl;
		CalibrateSensors();
	}
	void ImageClickedPointCallback(const geometry_msgs::PointStamped& in_clicked_point)
	{
		//图像点数已经比雷达点数大，因此用当前图像点覆盖上一图像点，
		//错选后可重新选择
		if(clicked_image_points_.size() - clicked_velodyne_points_.size() >0)
		{
			clicked_image_points_[clicked_image_points_.size()-1] 
				= cv::Point2f(in_clicked_point.point.x, in_clicked_point.point.y);
		}
		else
			clicked_image_points_.push_back(cv::Point2f(in_clicked_point.point.x,
		                                               in_clicked_point.point.y));

//		std::cout << cv::Point2f(in_clicked_point.point.x,
//		                         in_clicked_point.point.y) << std::endl << std::endl;

		CalibrateSensors();
	}
	
	//载入相机内参
	bool LoadCalibrationFile(const std::string& calibration_file)
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
		

		fs["CameraMat"] >> camera_instrinsics_;
		fs["DistCoeff"] >> distortion_coefficients_;
		fs["ImageSize"] >> image_size_;
		fs["DistModel"] >> dist_model_;
	
		return true;
	}
	
	void IntrinsicsCallback(const sensor_msgs::CameraInfo& in_message)
	{
		if (camera_instrinsics_.empty() || distortion_coefficients_.empty())
		{
			image_size_.height = in_message.height;
			image_size_.width = in_message.width;

			camera_instrinsics_ = cv::Mat(3, 3, CV_64F);
			for (int row = 0; row < 3; row++)
			{
				for (int col = 0; col < 3; col++)
				{
					camera_instrinsics_.at<double>(row, col) = in_message.K[row * 3 + col];
				}
			}

			distortion_coefficients_ = cv::Mat(1, 5, CV_64F);
			for (int col = 0; col < 5; col++)
			{
				distortion_coefficients_.at<double>(col) = in_message.D[col];
			}
			ROS_INFO("[%s] Camera Intrinsics Loaded", __APP_NAME__);
		}
	}
	
	

public:
	
	ROSCameraLidarApp():nh_private_("~"){}
	
	void Run()
	{
		std::string image_topic_str, points_raw_topic_str, clusters_topic_str, camera_info_topic_str;
		std::string name_space_str = ros::this_node::getNamespace();

		nh_private_.param<std::string>("image_src", image_topic_str, "/image_rectified");
		nh_private_.param<std::string>("camera_info_src", camera_info_topic_str, "/camera_info");

		if (name_space_str != "/")
		{
			if (name_space_str.substr(0, 2) == "//")
			{
				name_space_str.erase(name_space_str.begin());
			}
			image_topic_str = name_space_str + image_topic_str;
			camera_info_topic_str = name_space_str + camera_info_topic_str;
		}
		
		ROS_INFO("[%s] Subscribing to... %s",__APP_NAME__, camera_info_topic_str.c_str());
		subscriber_intrinsics_ = node_handle_.subscribe(camera_info_topic_str, 1, &ROSCameraLidarApp::IntrinsicsCallback, this);

		ROS_INFO("[%s] Subscribing to PointCloud ClickedPoint from RVIZ... /clicked_point",__APP_NAME__);
		subscriber_clicked_point_ = node_handle_.subscribe("/clicked_point", 1, &ROSCameraLidarApp::RvizClickedPointCallback, this);

		ROS_INFO("[%s] Subscribing to Image ClickedPoint from JSK ImageView2... %s/screenpoint",__APP_NAME__, image_topic_str.c_str());
		subscriber_image_point_ = node_handle_.subscribe(image_topic_str+"/screenpoint", 1, &ROSCameraLidarApp::ImageClickedPointCallback, this);
		ROS_INFO("[%s] ClickedPoint: %s",__APP_NAME__, (image_topic_str+"/screenpoint").c_str());

		ROS_INFO("[%s] Ready. Waiting for data...",__APP_NAME__);
		ros::spin();
		ROS_INFO("[%s] END",__APP_NAME__);
	}

};

int main(int argc, char **argv)
{
	ros::init(argc, argv, __APP_NAME__);

	ROSCameraLidarApp app;

	app.Run();

	return 0;
}

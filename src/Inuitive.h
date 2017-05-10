#ifndef INUITIVE_H
#define INUITIVE_H

#include <ros/ros.h>
#include <iostream>
#include <fstream>
#include <string>
#include <memory>

#include "InuSensor.h"
#include "InuSensorExt.h"
#include "DepthStream.h"
#include "VideoStream.h"
#include "WebCamStream.h"
#include "AuxStream.h"
#include "HeadStream.h"
#include "HandsStream.h"
#include "GazeStream.h"
#include "GeneralPurposeStream.h"

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <tf/transform_broadcaster.h>
#include "sensor_msgs/Imu.h"
#include "geometry_msgs/Quaternion.h"
#include "sensor_msgs/MagneticField.h"
#include <sensor_msgs/image_encodings.h>
#include "sensor_msgs/CameraInfo.h"
#include <camera_info_manager/camera_info_manager.h>
#include <sensor_msgs/distortion_models.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#define Rad2Deg 57.2957795130823208767981548141052
#define Deg2Rad 0.0174532922222222

class Inuitive {
public:
	Inuitive(ros::NodeHandle& n);
	~Inuitive();
	std::shared_ptr<InuDev::CInuSensorExt> inuSensor;
	bool Start();
	bool Stop();
	void SaveDepthFrams(std::shared_ptr<InuDev::CDepthStream> iStream,
			const InuDev::CImageFrame &iFrame, InuDev::CInuError retCode);
	void SaveCameraFrams(std::shared_ptr<InuDev::CWebCamStream> iStream,
			const InuDev::CImageFrame &iFrame, InuDev::CInuError retCode);
	void SaveAuxFrams(std::shared_ptr<InuDev::CAuxStream> iStream,
			const InuDev::CImuFrame &iFrame, InuDev::CInuError retCode);
	void SaveVideoFrams(std::shared_ptr<InuDev::CVideoStream> iStream,
			const InuDev::CVideoFrame &iFrame, InuDev::CInuError retCode);
	void SaveHeadFrams(std::shared_ptr<InuDev::CHeadStream> iStream,
			const InuDev::CHeadFrame &iFrame, InuDev::CInuError retCode);
	void SaveHandsFrams(std::shared_ptr<InuDev::CHandsStream> iStream,
			const InuDev::CHandsFrame &iFrame, InuDev::CInuError retCode);
	void SaveGazeFrams(std::shared_ptr<InuDev::CGazeStream> iStream,
			const InuDev::CGazeFrame &iFrame, InuDev::CInuError retCode);
	void SaveGeneralPurposeFrams(
			std::shared_ptr<InuDev::CGeneralPurposeStream> iStream,
			const InuDev::CGeneralFrame &iFrame, InuDev::CInuError retCode);
	ros::NodeHandle& nh;
	ros::Publisher depth_pub, camera_pub, videoR_pub, videoL_pub, imu_pub,
			mag_pub, quat_pub, pointcloud_pub;

	image_transport::CameraPublisher pub_color_;
	image_transport::CameraPublisher pub_depth_;
	image_transport::CameraPublisher pub_depth_raw_;

	boost::shared_ptr<camera_info_manager::CameraInfoManager>
			color_info_manager_, ir_info_manager_;

	ros::NodeHandle color_nh;
	image_transport::ImageTransport color_it;
	ros::NodeHandle depth_nh;
	image_transport::ImageTransport depth_it;
	ros::NodeHandle depth_raw_nh;
	image_transport::ImageTransport depth_raw_it;
	// unsigned int depth_frame_index,IR_left_frame_index,IR_right_frame_index,fisheye_frame_index;

	// param
	double DigitalGainRight_, DigitalGainLeft_, AnalogGainRight_,
			AnalogGainLeft_, ExposureTimeLeft_, ExposureTimeRight_;

private:
	Eigen::Matrix3d cib;
	Eigen::Quaternion<double> q, rk;
	Eigen::Vector3d gyro, angle, acc, mag, att;

	sensor_msgs::Imu imu_data;
	sensor_msgs::MagneticField magnet_data;
	geometry_msgs::Quaternion quat_data;
	geometry_msgs::Vector3 acc_msg, gyro_msg, mag_msg;

	unsigned int imu_seq, quat_seq, mag_seq, gyro_timestamp_pre, gyro_timestamp;
	bool gyro_flag, acc_flag, first_frame;
	bool depth_enable, video_enable, camera_enable, IMU_enable;
	bool head_enable, hands_enable, gaze_enable, generalPurpose_enable;
	std::shared_ptr<InuDev::CDepthStream> depthStream;
	std::shared_ptr<InuDev::CWebCamStream> cameraStream;
	std::shared_ptr<InuDev::CAuxStream> auxStream;
	std::shared_ptr<InuDev::CVideoStream> videoStream;
	std::shared_ptr<InuDev::CHeadStream> headStream;
	std::shared_ptr<InuDev::CHandsStream> handsStream;
	std::shared_ptr<InuDev::CGazeStream> gazeStream;
	std::shared_ptr<InuDev::CGeneralPurposeStream> generalPurposeStream;

	ros::Time time_stamp_;
    tf::Transform tr;
    tf::Quaternion q_tf;
    tf::TransformBroadcaster tf_broadcaster;
	std::string base_frame_id_;
	std::string depth_frame_id_;
	std::string color_frame_id_;
	std::string depth_optical_frame_id_;
	std::string color_optical_frame_id_;
	std::string ir_frame_id_;
	std::string ir2_frame_id_;
	const std::string DEFAULT_BASE_FRAME_ID = "camera_link";
	const std::string DEFAULT_DEPTH_FRAME_ID = "camera_depth_frame";
	const std::string DEFAULT_COLOR_FRAME_ID = "camera_rgb_frame";
	const std::string DEFAULT_DEPTH_OPTICAL_FRAME_ID = "camera_depth_optical_frame";
	const std::string DEFAULT_COLOR_OPTICAL_FRAME_ID = "camera_rgb_optical_frame";
	const std::string DEFAULT_IR_FRAME_ID = "camera_infrared_frame";
	const std::string DEFAULT_IR2_FRAME_ID = "camera_infrared2_frame";
	unsigned int depth_seq,rgb_seq;

	sensor_msgs::CameraInfoPtr Depth_camera_info,Rgb_camera_info;
	sensor_msgs::CameraInfoPtr getDepthCameraInfo(int width, int height,ros::Time stamp) const;
	sensor_msgs::CameraInfoPtr getRGBCameraInfo(int width, int height,ros::Time stamp) const;
	sensor_msgs::CameraInfoPtr Fisheye_info, Depth_info;
	InuDev::COpticalData OpticalData;
	void compute_attitude();
	void getOpticalParam();
	void publishTransforms();
	void SavePointcloud(cv::Mat depth,cv::Mat rgb);
};
#endif

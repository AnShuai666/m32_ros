/*
 * Inuitive.cpp
 *
 *  Created on: May 7, 2016
 *      Author: qfeel
 */
#include"Inuitive.h"

Inuitive::Inuitive(ros::NodeHandle& n) : nh(n), color_nh(n), depth_nh(n), depth_raw_nh(n), color_it(color_nh), depth_it(depth_nh), depth_raw_it(depth_raw_nh) {

    nh.param("DigitalGainRight", DigitalGainRight_, 30.0);
    nh.param("DigitalGainLeft", DigitalGainLeft_, 30.0);
    nh.param("AnalogGainRight", AnalogGainRight_, 0.0);
    nh.param("AnalogGainLeft", AnalogGainLeft_, 0.0);
    nh.param("ExposureTimeLeft", ExposureTimeLeft_, 1500.0);
    nh.param("ExposureTimeRight", ExposureTimeRight_, 1500.0);

    nh.param("depthEnable", depth_enable, true);
    nh.param("videoEnable", video_enable, true);
    nh.param("cameraEnable", camera_enable, true);
    nh.param("IMUEnable", IMU_enable, true);

    nh.param("headEnable", head_enable, true);
    nh.param("handsEnable", hands_enable, true);
    nh.param("gazeEnable", gaze_enable, true);
    nh.param("generalPurposeEnable", generalPurpose_enable, true);

    nh.param("base_frame_id", base_frame_id_, DEFAULT_BASE_FRAME_ID);
    nh.param("depth_frame_id", depth_frame_id_, DEFAULT_DEPTH_FRAME_ID);
    nh.param("color_frame_id", color_frame_id_, DEFAULT_COLOR_FRAME_ID);
    nh.param("depth_optical_frame_id", depth_optical_frame_id_, DEFAULT_DEPTH_OPTICAL_FRAME_ID);
    nh.param("color_optical_frame_id", color_optical_frame_id_, DEFAULT_COLOR_OPTICAL_FRAME_ID);
    nh.param("ir_frame_id", ir_frame_id_, DEFAULT_IR_FRAME_ID);
    nh.param("ir2_frame_id", ir2_frame_id_, DEFAULT_IR2_FRAME_ID);

    pub_color_ = color_it.advertiseCamera("/camera/rgb/image_raw", 10);
    pub_depth_raw_ = depth_raw_it.advertiseCamera("/camera/depth/image_raw", 10);
    pub_depth_ = depth_it.advertiseCamera("/camera/depth/image", 10);

    videoL_pub = n.advertise<sensor_msgs::Image>("/camera/ir/Left/image", 1000);
    videoR_pub = n.advertise<sensor_msgs::Image>("/camera/ir/Right/image", 1000);
    imu_pub = n.advertise<sensor_msgs::Imu>("/imu/imu", 1000);
    mag_pub = n.advertise<sensor_msgs::MagneticField>("/imu/mag", 1000);
    quat_pub = n.advertise<geometry_msgs::Quaternion>("/imu/quat", 1000);
    pointcloud_pub = n.advertise<sensor_msgs::PointCloud2>("/camera/depth/pointclouds", 100);
    imu_seq = 0;
    quat_seq = 0;
    mag_seq = 0;
    gyro_flag = false;
    acc_flag = false;
    cib << 1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0;
    q = {1.0, 0.0, 0.0, 0.0};
    first_frame = true;
    depth_seq=0;
    rgb_seq=0;
    Start();
}

Inuitive::~Inuitive(){

}
void Inuitive::publishTransforms(){
    ros::Time time_stamp = ros::Time::now();

     // transform base frame to depth frame
     tr.setOrigin(tf::Vector3(0.04622550, 0.00772170,0.02068530));
     tr.setRotation(tf::Quaternion(0, 0, 0, 1));
     tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, base_frame_id_, depth_frame_id_));

     // transform depth frame to depth optical frame
     tr.setOrigin(tf::Vector3(0,0,0));
     q_tf.setEuler( M_PI/2, 0.0, -M_PI/2 );
     tr.setRotation( q_tf );
     tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, depth_frame_id_, depth_optical_frame_id_));

     // transform base frame to color frame (these are the same)
     tr.setOrigin(tf::Vector3(0,0,0));
     tr.setRotation(tf::Quaternion(0, 0, 0, 1));
     tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, base_frame_id_, color_frame_id_));

     // transform color frame to color optical frame
     tr.setOrigin(tf::Vector3(0,0,0));
     q_tf.setEuler( M_PI/2, 0.0, -M_PI/2 );
     tr.setRotation( q_tf );
     tf_broadcaster.sendTransform(tf::StampedTransform(tr, time_stamp, color_frame_id_, color_optical_frame_id_));

}
void Inuitive::getOpticalParam(){

    inuSensor->GetOpticalData(OpticalData);
    ROS_INFO("OpticalData Version: %d",OpticalData.Version);
    ROS_INFO("OpticalData Left IR: Center [%lf, %lf], Focal:%lf",OpticalData.CenterL[0],OpticalData.CenterL[1],OpticalData.FocalL);
    ROS_INFO("OpticalData Right IR: Center [%lf, %lf], Focal:%lf",OpticalData.CenterR[0],OpticalData.CenterR[1],OpticalData.FocalR);
    ROS_INFO("OpticalData Baseline: %lf",OpticalData.BaseLine);
    ROS_INFO("OpticalData Sensor size (resolution) [%lf, %lf]",OpticalData.EffectiveSize[0],OpticalData.EffectiveSize[1]);
    ROS_INFO("OpticalData Correction for squint between left and right sensors: coefficients of linear fit <K0,K1> K0:%lf, K1:%lf",OpticalData.Squint[0],OpticalData.Squint[1]);
    if(!OpticalData.WebcamDataValid)
    	ROS_WARN("OpticalData Webcam invalid");
    ROS_INFO("OpticalData Webcam: Center [%lf, %lf], Focal [%lf, %lf]",OpticalData.WebcamCenter[0],OpticalData.WebcamCenter[1],OpticalData.WebcamFocal[0],OpticalData.WebcamFocal[1]);
    ROS_INFO("OpticalData WebcamTranslate: [%lf, %lf, %lf]",OpticalData.WebcamTranslate[0],OpticalData.WebcamTranslate[1],OpticalData.WebcamTranslate[2]);
    ROS_INFO("OpticalData WebcamRotate: [%lf, %lf, %lf]",OpticalData.WebcamRotate[0],OpticalData.WebcamRotate[1],OpticalData.WebcamRotate[2]);
    ROS_INFO("OpticalData WebcamDistortion: [%lf, %lf, %lf, %lf, %lf]",OpticalData.WebcamDistortion[0],OpticalData.WebcamDistortion[1],OpticalData.WebcamDistortion[2],OpticalData.WebcamDistortion[3],OpticalData.WebcamDistortion[4]);
    ROS_INFO("OpticalData TranslationUV(Internal use for fine tuning ): [%lf, %lf]",OpticalData.TranslationUV[0],OpticalData.TranslationUV[1]);

}
// Methods to get calibration parameters for the various cameras
sensor_msgs::CameraInfoPtr Inuitive::getDepthCameraInfo(int width, int height,ros::Time stamp) const
{
    sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
    info->header.frame_id = depth_optical_frame_id_;
    info->header.stamp = stamp;
    info->header.seq = depth_seq;
    info->width  = width;
    info->height = height;

    // No distortion
    info->D.resize(5, 0.0);
    info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    info->D[0] = -0.226826;
    info->D[1] = 0.041337;
    info->D[2] = 0.001269;
    info->D[3] = -0.004055;
    info->D[4] =  0.000000;
    // Simple camera matrix: square pixels (fx = fy), principal point at center
    info->K.assign(0.0);
    info->K[0] = 556.813782; //fx
    info->K[4] = 556.813782; // fy
    info->K[2] = 312.000000;     // cx
    info->K[5] =  232.000000;     // cy
    info->K[8] = 1.0;

    // No separate rectified image plane, so R = I
    info->R.assign(0.0);
    info->R[0] = info->R[4] = info->R[8] = 1.0;

    // Then P=K(I|0) = (K|0)
    info->P.assign(0.0);
    info->P[0]  = 556.813782; //fx
    info->P[5] = 556.813782; // fy
    info->P[2]  = 312.000;     // cx
    info->P[6]  = 232.00;     // cy
    info->P[10] = 1.0;

    return info;
}
sensor_msgs::CameraInfoPtr Inuitive::getRGBCameraInfo(int width, int height,ros::Time stamp) const
{
    sensor_msgs::CameraInfoPtr info = boost::make_shared<sensor_msgs::CameraInfo>();
    info->header.frame_id = color_optical_frame_id_;
    info->header.stamp = stamp;
    info->header.seq = rgb_seq;
    info->width  = width;
    info->height = height;

    // No distortion
    info->D.resize(5, 0.0);
    info->distortion_model = sensor_msgs::distortion_models::PLUMB_BOB;
    info->D[0] = OpticalData.WebcamDistortion[0];
    info->D[1] = OpticalData.WebcamDistortion[1];
    info->D[2] = OpticalData.WebcamDistortion[2];
    info->D[3] = OpticalData.WebcamDistortion[3];
    info->D[4] = OpticalData.WebcamDistortion[4];
    // Simple camera matrix: square pixels (fx = fy), principal point at center
    info->K.assign(0.0);
    info->K[0] = 732.052429; //fx
    info->K[4] = 732.052429; // fy
    info->K[2] = 280.800659;     // cx
    info->K[5] = 245.115341;     // cy
    info->K[8] = 1.0;

    // No separate rectified image plane, so R = I
    info->R.assign(0.0);
    info->R[0] = info->R[4] = info->R[8] = 1.0;

    // Then P=K(I|0) = (K|0)
    info->P.assign(0.0);
    info->P[0]  = 732.052429; //fx
    info->P[5] = 732.052429; // fy
    info->P[2]  = 280.800659;     // cx
    info->P[6]  = 245.115341;     // cy
    info->P[10] = 1.0;

    return info;
}
void Inuitive::compute_attitude() {
    double interval = (double)(gyro_timestamp - gyro_timestamp_pre) / 1000000000;
    gyro_timestamp_pre = gyro_timestamp;
    angle = gyro * interval;
    double u = sqrt(angle.x() * angle.x() + angle.y() * angle.y() +
                    angle.z() * angle.z());
    if (abs(u) < 0.001) u = 0.0001;
    double ac = cos(u / 2);
    double as = sin(u / 2) / u;

    rk = {ac, as * angle.x(), as * angle.y(), as * angle.z()};
    q = q * rk;
    q = q.normalized();
    cib = q.matrix();
}

void Inuitive::SaveDepthFrams(
    std::shared_ptr<InuDev::CDepthStream> iStream,  // Parent Stream
    const InuDev::CImageFrame &iFrame,              // Acquired frame
    InuDev::CInuError
    retCode)  // Error code (eOK if frame was successfully acquired)
{
    sensor_msgs::CameraInfoPtr info;
    time_stamp_ = ros::Time::now();
    depth_seq++;
    if (retCode != InuDev::eOK) {
        std::cout << "Error in receiving frame: " << std::hex << int(retCode) << " "
                  << std::string(retCode) << std::endl;
        return;
    }

//     cv::namedWindow("depth", cv::WINDOW_AUTOSIZE);

    cv::Mat depth(iFrame.Height(), iFrame.Width(), CV_16UC1, (void *)iFrame.GetData(), iFrame.Width() * 2);

    for (int i = 0; i < iFrame.Height() * iFrame.Width() * 2; i += 2) {
        unsigned char a = depth.data[i];
        depth.data[i] = depth.data[i + 1];
        depth.data[i + 1] = a;
    }

//     cv::imshow("depth_raw", depth);   cv::waitKey(1);
    cv::Mat depth_map(iFrame.Height(), iFrame.Width(), CV_32FC1);
    unsigned int tmp_depth;
    for (int i = 0; i < depth.rows; i++) {
        for (int j = 0; j < depth.cols; j++) {
            tmp_depth = (unsigned int)(*(iFrame.GetData() + 2 * (i * depth.cols + j)))* 256 + (unsigned int)(*(iFrame.GetData() + 2 * (i * depth.cols + j) + 1)) ;
            depth_map.at<float>(i, j) = (float)tmp_depth / 1000.0;
            if(depth_map.at<float>(i, j)>4)
            	depth_map.at<float>(i, j)=0;
        }
    }

    cv_bridge::CvImage depth_msg;
    depth_msg.encoding = sensor_msgs::image_encodings::TYPE_32FC1;

    depth_msg.image = depth_map;
    depth_msg.header.frame_id = depth_optical_frame_id_;
    depth_msg.header.stamp = time_stamp_;
    Depth_camera_info=getDepthCameraInfo(iFrame.Width(), iFrame.Height(),time_stamp_);
    pub_depth_.publish(depth_msg.toImageMsg(), Depth_camera_info);
    depth_msg.encoding = sensor_msgs::image_encodings::TYPE_16UC1;
    depth_msg.image = depth;
    pub_depth_raw_.publish(depth_msg.toImageMsg(), Depth_camera_info);
    publishTransforms();
}
void Inuitive::SaveCameraFrams(
    std::shared_ptr<InuDev::CWebCamStream> iStream,  // Parent Stream
    const InuDev::CImageFrame &iFrame,               // Acquired frame
    InuDev::CInuError
    retCode)  // Error code (eOK if frame was successfully acquired)
{
    sensor_msgs::CameraInfoPtr info;
    time_stamp_ = ros::Time::now();
    rgb_seq++;
    if (retCode != InuDev::eOK) {
        std::cout << "Error in receiving frame: " << std::hex << int(retCode) << " "
                  << std::string(retCode) << std::endl;
        return;
    }
    cv::Mat video_rgba8,video_rgb8;
    cv::Mat camera_raw(iFrame.Height(), iFrame.Width(), CV_8UC4, (void *)iFrame.GetData(), iFrame.Width() * 4);
    cv::cvtColor(camera_raw, video_rgb8, CV_BGRA2RGB);

    cv_bridge::CvImage video_msg;
    video_msg.encoding = sensor_msgs::image_encodings::RGB8;
    video_msg.image = video_rgb8;
    video_msg.header.frame_id = color_optical_frame_id_;
    video_msg.header.stamp = time_stamp_;
    // camera_pub.publish(video_msg.toImageMsg());
    Rgb_camera_info=getRGBCameraInfo(iFrame.Width(), iFrame.Height(),time_stamp_);
    pub_color_.publish(video_msg.toImageMsg(), Rgb_camera_info);
}
void Inuitive::SaveVideoFrams(
    std::shared_ptr<InuDev::CVideoStream> iStream,  // Parent Stream
    const InuDev::CVideoFrame &iFrame,               // Acquired frame
    InuDev::CInuError
    retCode)  // Error code (eOK if frame was successfully acquired)
{
    sensor_msgs::CameraInfoPtr info;
    if (retCode != InuDev::eOK) {
        std::cout << "Error in receiving frame: " << std::hex << int(retCode) << " "
                  << std::string(retCode) << std::endl;
        return;
    }
    const InuDev::CImageFrame* LFrame = iFrame.GetLeftFrame();
    const InuDev::CImageFrame* RFrame = iFrame.GetRightFrame();

    // cv::namedWindow("videoL", cv::WINDOW_AUTOSIZE);
    // cv::namedWindow("videoR", cv::WINDOW_AUTOSIZE);

    cv::Mat videoL(LFrame->Height(), LFrame->Width(), CV_8UC1);
    cv::Mat videoR(RFrame->Height(), RFrame->Width(), CV_8UC1);

    for (long k = 0; k < LFrame->BufferSize() / 4; k++) {
        videoL.data[k] = *((unsigned char *)LFrame->GetData() + 4 * k);
        videoR.data[k] = *((unsigned char *)RFrame->GetData() + 4 * k);
    }

    cv_bridge::CvImage video_msg;
    video_msg.encoding = sensor_msgs::image_encodings::TYPE_8UC1;
    video_msg.image = videoR;
    video_msg.header.frame_id = std::string("/openni_ir_R_optical_frame");
    videoR_pub.publish(video_msg.toImageMsg());
    video_msg.image = videoL;
    video_msg.header.frame_id = std::string("/openni_ir_L_optical_frame");
    videoL_pub.publish(video_msg.toImageMsg());
}
void Inuitive::SaveAuxFrams(
    std::shared_ptr<InuDev::CAuxStream> iStream,  // Parent Stream
    const InuDev::CImuFrame &iFrame,              // Acquired frame
    InuDev::CInuError
    retCode)  // Error code (eOK if frame was successfully acquired)
{
    if (retCode != InuDev::eOK) {
        std::cout << "Error in receiving frame: " << std::hex << int(retCode) << " "
                  << std::string(retCode) << std::endl;
        return;
    }
    std::map<InuDev::EAuxType, InuDev::CPoint3D> SensorsData = iFrame.SensorsData;
    for (std::map<InuDev::EAuxType, InuDev::CPoint3D>::iterator it =
                SensorsData.begin();
            it != SensorsData.end(); ++it) {
        if (it->first == InuDev::EAuxType::eAccelarometer) {
            acc_msg.x = it->second.X();
            acc_msg.y = it->second.Y();
            acc_msg.z = it->second.Z();
            acc_flag = true;
        }

        if (it->first == InuDev::EAuxType::eGyroscope) {
            gyro_msg.x = it->second.X();
            gyro_msg.y = it->second.Y();
            gyro_msg.z = it->second.Z();
            gyro = Eigen::Vector3d(it->second.X(), it->second.Y(), it->second.Z());
            gyro_flag = true;
            if (first_frame) {
                first_frame = false;
                gyro_timestamp_pre = iFrame.Timestamp;
                gyro_timestamp = iFrame.Timestamp;
            } else {
                gyro_timestamp = iFrame.Timestamp;
                compute_attitude();
            }
        }
        if (it->first == InuDev::EAuxType::eMagnetometer) {
            mag_msg.x = it->second.X();
            mag_msg.y = it->second.Y();
            mag_msg.z = it->second.Z();

            magnet_data.header.seq = mag_seq;
            mag_seq++;
            magnet_data.header.stamp = ros::Time::now();  // iFrame.Timestamp;
            magnet_data.magnetic_field = mag_msg;
            mag_pub.publish(magnet_data);
        }
        if (gyro_flag & acc_flag) {
            gyro_flag = false;
            acc_flag = false;
            imu_data.header.seq = imu_seq;
            imu_seq++;
            imu_data.header.stamp = ros::Time::now();
            quat_data.x = q.x();
            quat_data.y = q.y();
            quat_data.z = q.z();
            quat_data.w = q.w();
            imu_data.orientation = quat_data;
            imu_data.angular_velocity = gyro_msg;
            imu_data.linear_acceleration = acc_msg;
            imu_pub.publish(imu_data);
        }
    }
}
void Inuitive::SaveHeadFrams(std::shared_ptr<InuDev::CHeadStream> iStream,
                    const InuDev::CHeadFrame &iFrame, InuDev::CInuError retCode){
}
void Inuitive::SaveHandsFrams(std::shared_ptr<InuDev::CHandsStream> iStream,
                    const InuDev::CHandsFrame &iFrame, InuDev::CInuError retCode){
	for(int i=0;i<2;i++)
		if(iFrame.Hands[i].Side != InuDev::CHandData::eNotValid)
			ROS_INFO("palmCenter:[%lf,%lf]",iFrame.Hands[i].PalmCenter.X(),iFrame.Hands[i].PalmCenter.Y());
}
void Inuitive::SaveGazeFrams(std::shared_ptr<InuDev::CGazeStream> iStream,
                    const InuDev::CGazeFrame &iFrame, InuDev::CInuError retCode){

}
void Inuitive::SaveGeneralPurposeFrams(std::shared_ptr<InuDev::CGeneralPurposeStream> iStream,
                    const InuDev::CGeneralFrame &iFrame, InuDev::CInuError retCode){
}
bool Inuitive::Start() {
    // Creation of CInuSensor object (Inuitive's sensor representation)
    inuSensor = InuDev::CInuSensor::Create();

    // Initiate the sensor - it must be call before any access to the sensor.
    // Sensor will start working in low power.
    InuDev::CSensorParams iSensorParams;
    iSensorParams.FPS = 30.0;
    iSensorParams.SensorRes = InuDev::eFull;
    InuDev::CInuError retCode = inuSensor->Init();
    if (retCode != InuDev::eOK) {
        ROS_ERROR("Failed to connect to Inuitive Sensor. Error: %X", (int)retCode);
        return false;
    }
    ROS_INFO("Connected to Sensor");

    // Start acquiring frames - it must be call before starting acquiring any type
    // of frames (depth, video, head, etc.)
    retCode = inuSensor->Start();
    if (retCode != InuDev::eOK) {
        ROS_INFO("Failed to start to Inuitive Sensor.");
        return false;
    }
    ROS_INFO("Sensor is started");

    getOpticalParam();
    ROS_INFO("Stream is initializing");
    if(depth_enable){
    	depthStream = inuSensor->CreateDepthStream();
		retCode = depthStream->Init();//InuDev::CDepthStream::eDepthRegistration

		// InuDev::CSensorControlParams cam_para;
		// inuSensor->GetSensorControlParams(cam_para, 0);
		// cam_para.DigitalGainRight = (unsigned int)DigitalGainRight_;
		// cam_para.DigitalGainLeft = (unsigned int)DigitalGainLeft_;
		// cam_para.AnalogGainRight = (unsigned int)AnalogGainRight_;
		// cam_para.AnalogGainLeft = (unsigned int)AnalogGainLeft_;
		// cam_para.ExposureTimeLeft = (unsigned int)ExposureTimeLeft_;
		// cam_para.ExposureTimeRight = (unsigned int)ExposureTimeRight_;
		// inuSensor->SetSensorControlParams(cam_para, 0);

		//retCode = depthStream->SetPostProcess(InuDev::eExtraSmoothMode);//eFastMode
		retCode = depthStream->Start();
		retCode = depthStream->Register(

				std::bind(&Inuitive::SaveDepthFrams, this,
						std::placeholders::_1, std::placeholders::_2,
						std::placeholders::_3));
		if (retCode != InuDev::eOK) {
			ROS_ERROR("Depth register error: %X", (int )retCode);
			return false;
		}
    }
    if(camera_enable){
    	cameraStream = inuSensor->CreateWebCamStream();
    	retCode = cameraStream->Init();
    	retCode = cameraStream->Start();
        retCode = cameraStream->Register(
                      std::bind(&Inuitive::SaveCameraFrams, this, std::placeholders::_1,
                                std::placeholders::_2, std::placeholders::_3));
        if (retCode != InuDev::eOK) {
            ROS_ERROR("Camera register error: %X", (int)retCode);
            return false;
        }
    }
    if(video_enable){
    	videoStream = inuSensor->CreateVideoStream();
    	retCode = videoStream->Init();
    	retCode = videoStream->Start();
    	retCode = videoStream->Register(std::bind(&Inuitive::SaveVideoFrams, this, std::placeholders::_1,
                                    std::placeholders::_2, std::placeholders::_3));
		if (retCode != InuDev::eOK) {
			ROS_ERROR("Video register error: %X", (int)retCode);
			return false;
		}
	}
    if(IMU_enable){
    	auxStream = inuSensor->CreateAuxStream();
    	retCode = auxStream->Init();
    	retCode = auxStream->Start();
        retCode = auxStream->Register((InuDev::CAuxStream::ImuCallbackFunction)
                      std::bind(&Inuitive::SaveAuxFrams, this, std::placeholders::_1,
                                std::placeholders::_2, std::placeholders::_3));
        if (retCode != InuDev::eOK) {
            ROS_ERROR("Aux register error: %X", (int)retCode);
            return false;
        }
    }
    if(head_enable){
    	headStream = inuSensor->CreateHeadStream();
    	retCode = headStream->Init();
    	retCode = headStream->Start();
        retCode = headStream->Register(
                      std::bind(&Inuitive::SaveHeadFrams, this, std::placeholders::_1,
                                std::placeholders::_2, std::placeholders::_3));
        if (retCode != InuDev::eOK) {
            ROS_ERROR("Head register error: %X", (int)retCode);
            return false;
        }
    }
	if (hands_enable) {
		handsStream = inuSensor->CreateHandsStream();
    	retCode = handsStream->Init();
    	retCode = handsStream->Start();
        retCode = handsStream->Register(
                      std::bind(&Inuitive::SaveHandsFrams, this, std::placeholders::_1,
                                std::placeholders::_2, std::placeholders::_3));
        if (retCode != InuDev::eOK) {
            ROS_ERROR("hands register error: %X", (int)retCode);
            return false;
        }
	}
	if (gaze_enable) {
		gazeStream = inuSensor->CreateGazeStream();
    	retCode = gazeStream->Init();
    	retCode = gazeStream->Start();
//    	retCode = gazeStream->StartCalibration();
        retCode = gazeStream->Register(
                      std::bind(&Inuitive::SaveGazeFrams, this, std::placeholders::_1,
                                std::placeholders::_2, std::placeholders::_3));
        if (retCode != InuDev::eOK) {
            ROS_ERROR("gaze register error: %X", (int)retCode);
            return false;
        }
	}
	if (generalPurpose_enable) {
		generalPurposeStream = inuSensor->CreateGeneralPurposeStream();
    	retCode = generalPurposeStream->Init();
    	retCode = generalPurposeStream->Start();
        retCode = generalPurposeStream->Register(
                      std::bind(&Inuitive::SaveGeneralPurposeFrams, this, std::placeholders::_1,
                                std::placeholders::_2, std::placeholders::_3));
        if (retCode != InuDev::eOK) {
            ROS_ERROR("gpstream register error: %X", (int)retCode);
            return false;
        }
	}
    ROS_INFO("Data is publishing...");

    return true;

}


bool Inuitive::Stop() {
    //unregister stream
    if (depth_enable){
    	depthStream->Register(nullptr);
    	depthStream->Stop();
    	depthStream->Terminate();
    	ROS_INFO("Depth Stream was finalized");
    }
    if(camera_enable){
    	cameraStream->Register(nullptr);
    	cameraStream->Stop();
    	cameraStream->Terminate();
    	ROS_INFO("Camera Stream was finalized");
    }
    if(video_enable){
    	videoStream->Register((InuDev::CVideoStream::CallbackFunction)nullptr);
    	videoStream->Stop();
    	videoStream->Terminate();
    	ROS_INFO("Video Stream was finalized");
	}
    if(IMU_enable){
		auxStream->Register((InuDev::CAuxStream::ImuCallbackFunction) nullptr);
		auxStream->Stop();
		auxStream->Terminate();
    	ROS_INFO("IMU Stream was finalized");
    }
    if(head_enable){
    	headStream->Register((InuDev::CHeadStream::CallbackFunction) nullptr);
    	headStream->Stop();
    	headStream->Terminate();
    	ROS_INFO("Head Stream was finalized");
    }
    if(hands_enable){
    	handsStream->Register((InuDev::CHandsStream::CallbackFunction) nullptr);
    	handsStream->Stop();
    	handsStream->Terminate();
    	ROS_INFO("Hands Stream was finalized");
    }
    if(gaze_enable){
    	gazeStream->Register((InuDev::CGazeStream::CallbackFunction) nullptr);
    	gazeStream->Stop();
    	gazeStream->Terminate();
    	ROS_INFO("Gaze Stream was finalized");
    }
    if(generalPurpose_enable){
    	generalPurposeStream->Register((InuDev::CGeneralPurposeStream::CallbackFunction) nullptr);
    	generalPurposeStream->Stop();
    	generalPurposeStream->Terminate();
    	ROS_INFO("GP Stream was finalized");
    }
    // Stop frames acquisition (of any type)
    inuSensor->Stop();
    ROS_INFO("Sensor was stopped");

    inuSensor->Terminate();
    ROS_INFO("Disconnected from Sensor");

    return true;
}

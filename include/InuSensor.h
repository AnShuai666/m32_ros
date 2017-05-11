/*
 * File - Provider.h
 *
 * This file is part of the Inuitive SDK
 *
 * Copyright (C) 2014 All rights reserved to Inuitive  
 *
 */

#ifndef __INUSENSOR_H__
#define __INUSENSOR_H__

#include "StreamsExport.h"
#include "InuError.h"
#include "BaseStream.h"
#include "InuDefs.h"

#include <map>
#include <functional>
#include <stdint.h>

namespace InuDev 
{
    // Forward decelerations
    class CDepthStream;
    class CVideoStream;
    class CHeadStream;
    class CHandsStream;
    class CSpeechRecognitionStream;
    class CGazeStream;
    class CGazeCalibrationStream;
    class CFaceRecognitionStream;
    class CFaceEnrollmentStream;
    class CFaceExpressionsStream;
    class CWebCamStream;
    class CAuxStream;
    class CGeneralPurposeStream;
    class CAudioStream;
    class CFeaturesTrackingStream;
    class CSlamStream;

    ///////////////////////////////////////////////////////////////////////
    /// \brief    Information about SW and HW versions
    ///////////////////////////////////////////////////////////////////////
    struct CEntityVersion
    {
        /// \brief    Each version entity has a unique identification number
        enum EEntitiesID
        {
			eSerialNumber,			///<  Device Serial Number 
			eModelNumber,			///<  Device Model Number
			ePartNumber,			///<  Device Part Number
			eRevisionNumber,		///<  Device Revision Number
			eMasterID,				///<  Master sensor ID
			eSlaveID,				///<  Slave sensor ID
			eFWVersion,				///<  Firmware
			eHWVersion,				///<  Hardware version, returned values are defined by EHWVersion enumerator 
			eStreams,				///<  InuStreams.dll 
			eServices,				///<  InuService.exe
			eHWRevision,			///<  Hardware revision
			eCalibrationRevision,   ///<  Calibration data revision
			eBootfixTimestamp,		///<  Boot fix time stamp
			eBootfixVersion			///<  Boot fix Version
        };

        /// \brief    Version Entity ID
        EEntitiesID ID;

        /// \brief    Name of this entity
        std::string Name;

        /// \brief    Version number 
        unsigned int VersionNum;

        /// \brief    Version description 
        std::string VersionName;
    };

    ///////////////////////////////////////////////////////////////////////
    /// \brief    HW revisions (provided by CEntityVersion::eHWRevision) 
    ///////////////////////////////////////////////////////////////////////
    enum EHWVersion
    {
        eA0  = 0,          ///<  First NU3000 generation 
        eB0 = 1,           ///<  Second NU3000 generation 
    }; 

	///////////////////////////////////////////////////////////////////////
	/// \brief    All types of cameras that can be assembled in InuSensor
	///////////////////////////////////////////////////////////////////////
	enum ECameraName
	{
		eVideo = 0,				///<  Stereo camera which is used for Depth calculation 
		eWebCam = 2,			///<  RGB or Fisheye camera
		eAllCameras = 100       ///<  limited to 255
	};

	///////////////////////////////////////////////////////////////////////
	/// \brief    All possible states of connection with Sensor 
	///////////////////////////////////////////////////////////////////////
	enum EConnectionState
	{
		eUnknownConnectionState = 0,	///<  Initial state, connection has not been established
		eConnected = 1,					///<  Sensor is connected 
		eDisconnected = 5,				///<  No Sensor is connected
		eServiceDisconnected = 4,		///<  Can't communicate with InuService
	};

	///////////////////////////////////////////////////////////////////////
	/// \brief    Configuration of current connected InuSesnor  (read only).
	///////////////////////////////////////////////////////////////////////
	struct CSensorConfiguration
	{
		/// \brief    Video (Stereo camera) Image size that is provided by the this sensor [Width,Height] according to channelID. 
		std::map<unsigned int, CPoint<2, unsigned int>> VideoSize;

		/// \brief    Depth Image size that is provided by the this sensor [Width,Height] according to channelID.  
		std::map<unsigned int, CPoint<2, unsigned int>> DepthSize;

		/// \brief    Webcam Image size that is provided by this sensor [Width,Height]. 
		std::map<unsigned int, CPoint<2, unsigned int>> WebCamSize;
	};

	///////////////////////////////////////////////////////////////////////
    /// \brief    All resolutions supported by Inuitive Sensor
    ///////////////////////////////////////////////////////////////////////
    enum ESensorResolution
    {
		eDefaultResolution = 0,		///< Sensor default resolutions
		eBinning = 1,				///< Sensor's binning mode (reduced resolution provided by sensor)
        eUserDefined = 2,			///< User defined resolution 
        eFull = 3,					///< Full sensor resolution  
        eAlternate = 4,				///< Alternating resolutions
    }; 


    ///////////////////////////////////////////////////////////////////////
    /// \brief    Sensor parameters 
    ///////////////////////////////////////////////////////////////////////
    struct CSensorParams
    { 
		static const int USE_DEFAULT_FPS = -1;

        /// \brief    Sensor Resolution
        ESensorResolution SensorRes;   

        /// \brief    Frame rate (number of frames per second)
        float FPS;         

		/// \brief    Default constructor which defines the default FPS and resolution
        CSensorParams() : SensorRes(eDefaultResolution), FPS(float(USE_DEFAULT_FPS)) {}
    };

    ///////////////////////////////////////////////////////////////////////
    /// \brief   Optical Data of connected InuSensor 
    ///
    /// Intrinsic & Extrinsic parameters of Left, Right and Webcam sensors
    ///////////////////////////////////////////////////////////////////////
    struct COpticalData
    {
        // Structure version: 3 and newer contains valid Webcam data
        uint32_t Version;     

        /// \brief    Focal Length of the left sensor (Intrinsic parameter)
        float FocalL;

        /// \brief    Optical center <X,Y> of the left sensor (Intrinsic parameter)
        CPoint2D  CenterL;   

        /// \brief    Focal Length of the Right sensor (Intrinsic parameter)
        float FocalR;            

        /// \brief    Optical center <X,Y> of the right sensor (Intrinsic parameter)
        CPoint2D CenterR;   

        /// \brief    Distance between sensors (Extrinsic parameter)
        float BaseLine;

        /// \brief   Sensor size (resolution) <X,Y>
        CPoint2D  EffectiveSize;        

        /// \brief   Correction for squint between left and right sensors: coefficients of linear fit <K0,K1>
        CPoint2D  Squint;

        /// Webcam data - valid only if WebcamDataValid is true 

        /// \brief    Focal Length <X,Y> of the Webcam sensor  (Intrinsic parameter)
        CPoint2D WebcamFocal;

        /// \brief    Optical center <X,Y> of the Webcam sensor  (Intrinsic parameter)
        CPoint2D WebcamCenter;

        /// \brief    Internal use for fine tuning 
        CPoint2D TranslationUV;   

        /// \brief    Translation of Webcam sensor relates to Depth image <X,Y,Z> (Extrinsic parameter) 
        float WebcamTranslate[3];

        /// \brief    Rotation of Webcam sensor relates to Depth image <X,Y,Z> (Extrinsic parameter) 
        float WebcamRotate[3];

        /// \brief    Webcam Distortion constants <K1,K2,K3,K4,K5> (Intrinsic parameter)
        float WebcamDistortion[5];              

        /// \brief    true if and only if Webcam parameters are valid
        bool  WebcamDataValid; //  from version 3

        COpticalData() : Version(3) {}
    };
   
    ///////////////////////////////////////////////////////////////////////
    /// \brief    Exposure parameters  
    ///////////////////////////////////////////////////////////////////////
    struct CExposureParams 
    {
        /// \brief    Exposure Time of the Right sensor
        uint32_t    ExposureTimeRight;

        /// \brief    Exposure Time of the Left sensor
        uint32_t    ExposureTimeLeft;

        /// \brief    Digital Gain of the Right sensor
        uint32_t    DigitalGainRight;

        /// \brief    Digital Gain of the Left sensor
        uint32_t    DigitalGainLeft;

        /// \brief    Analog Gain of the Right sensor
        uint32_t    AnalogGainRight;

        /// \brief    Analog Gain of the Left sensor
        uint32_t    AnalogGainLeft;


		/// \brief    Default constructor which reset all exposure parameters
		CExposureParams() :
			ExposureTimeRight(0),
			ExposureTimeLeft(0),
			DigitalGainRight(0),
			DigitalGainLeft(0),
			AnalogGainRight(0),
			AnalogGainLeft(0)
			{
			}
    };

	///////////////////////////////////////////////////////////////////////
	/// \brief    ROI (region of interest) for automatic sensor control  algorithm 
	///////////////////////////////////////////////////////////////////////
	struct CSensorControlROIParams
	{
		/// \brief    If true then ROI is applied in automatic sensor control
		bool UseROI;

		/// \brief    Top left corner of ROI
		CPoint<2, unsigned int> ROITopLeft;

		/// \brief    Bottom right corner of ROI
		CPoint<2, unsigned int> ROIBottomRight;

		/// \brief    Default constructor, when ROITopLeft=ROIBottomRight=0 then ROI is not apply 
		CSensorControlROIParams() : UseROI(false), ROITopLeft(0, 0), ROIBottomRight(0, 0) {}
	};

    ///////////////////////////////////////////////////////////////////////
    /// \brief    Sensor control parameters 
    ///////////////////////////////////////////////////////////////////////
    struct CSensorControlParams :public CExposureParams
    {
        
        /// \brief    If this flag is true then automatic algorithm for gain and exposure time adjustment (AGC) is activated. 
        bool AutoControl;

		/// \brief     ROI of each sensor
		CSensorControlROIParams Params[2];
        
		/// \brief    Default constructor, automatic sensor control is on 
		CSensorControlParams() : AutoControl(true){ }
    };

	///////////////////////////////////////////////////////////////////////
	/// \brief    All projectors that can be assembled in InuSensor
	///////////////////////////////////////////////////////////////////////
	enum EProjectors
	{
		ePatterns = 0,          ///< Main projector 
		eNumOfProjectors		///< Apply to all assembled projectors  
	};

    ///////////////////////////////////////////////////////////////////////
    /// \brief    Projector intensity levels
    ///////////////////////////////////////////////////////////////////////
    enum EProjectorLevel
    {
        eOff = 0,                          ///< Turn projector off
        eLow,                              ///< Low Projector intensity               
        eHigh                             ///< High Projector intensity
     };

    ////////////////////////////////////////////////////////////////////////
    /// \brief    Represents Inuitive's Sensor
    ///
    /// Role: Enables control of the sensor and generates all kinds of InuDev streams
    ///
    /// Responsibilities: 
    ///      1. Singleton object
    ///      2. Derives CBaseStream interface
    ///      3. Creates all kinds of InuDev streams
	///      4. Provides information about InuSensor connection state 
	///      5. Sensor control 
    ///      5. Get and Set different features of InuSensor
    ///
    ////////////////////////////////////////////////////////////////////////
    class CInuSensor : public CBaseStream
    {
    public:

        /// \brief    Prototype of callback function which is used by the Register method.
        ///
        /// This function is invoked any time a frame is ready, or if an error occurs. It provides information about InuSensor connection state
		/// The callback function arguments are: caller stream object, received Sensor state frame and result code.
        typedef std::function<void(std::shared_ptr<CInuSensor>, const EConnectionState&,  CInuError)> CallbackFunction;

        virtual ~CInuSensor() {} 

        /// \brief    Access to the Singleton object
        INUSTREAMS_API static std::shared_ptr<CInuSensor>  Create();

		// Generate all kinds of InuDev streams 
		virtual std::shared_ptr<CDepthStream> CreateDepthStream(uint32_t iChannelID = 0) = 0;
		virtual std::shared_ptr<CVideoStream> CreateVideoStream(uint32_t iChannelID = 0) = 0;
		virtual std::shared_ptr<CHeadStream>  CreateHeadStream(uint32_t iChannelID = 0) = 0;
		virtual std::shared_ptr<CHandsStream> CreateHandsStream(uint32_t iChannelID = 0) = 0;
		virtual std::shared_ptr<CGazeStream>  CreateGazeStream(uint32_t iChannelID = 0) = 0;
		virtual std::shared_ptr<CWebCamStream> CreateWebCamStream(uint32_t iChannelID = 0) = 0;
		virtual std::shared_ptr<CAuxStream>   CreateAuxStream(uint32_t iChannelID = 0) = 0;
        virtual std::shared_ptr<CGeneralPurposeStream>   CreateGeneralPurposeStream(uint32_t iChannelID = 0) = 0;
        virtual std::shared_ptr<CAudioStream>   CreateAudioStream(uint32_t iChannelID = 0) = 0;
        virtual std::shared_ptr<CFeaturesTrackingStream>   CreateFeaturesTrackingStream(uint32_t iChannelID = 0) = 0;
        virtual std::shared_ptr<CSlamStream>  CreateSlamStream(uint32_t iChannelID = 0) = 0;

        /// \brief    Derived interface, Init without sensor initial parameters
        ///
        /// When init is invoked with no parameters then it is started with default Resolution and FPS
        /// that match to current connected InuSensor. 
        /// After invoking Init method the sensor is still in low power consumption. 
        using CBaseStream::Init;

        /// \brief    Service initialization.
        /// 
        /// Invoked once before initialization of any other InuDev stream.
        /// After invoking Init method the sensor is still in low power consumption. 
        /// \param[in] iSensorParams    Initialized the sensor with these input parameters. It will be set to all assembeled cameras.
        /// \param[in] iSensorID    Unique ID of the sensor that should be accessed. Tcp/ip address for remote sensor.  
        /// \return CInuError    Operation status which indicates on success or failure.
        virtual CInuError Init(const CSensorParams& iSensorParams) = 0;

        /// \brief    Get the connection state of the sensor. 
        /// \return EConnectionState    
        virtual EConnectionState GetConnectionState() const = 0;
        
        /// \brief    Registration for receiving InuSensor state notifications (push). 
        /// 
        /// The provided callback function is called only when the sensor state is changed. 
        /// \param[in] iCallback    Callback function which is invoked whenever the sensor state is changed.
        /// \return CInuError    Error code, InDev::eOK if operation successfully completed.
        virtual CInuError Register(CallbackFunction iCallback) = 0;

		/// \brief		SW reset of InuSensor, it resets both SW and HW.
		/// \return CInuError If failed to reset the sensor 
		virtual CInuError Reset() = 0;
		
		/// \brief    Get current initialized  parameters. It should be used when other client application might be running and initialized InuSensor. 
		/// \param[out] oParams    Retrieved sensor parameters.
		/// \param[in] iCameraName    Parameters of which camera to retrieve, eAllCameras will return with error.
		/// \return CInuError    Error code, InDev::eOK if operation successfully completed.
		virtual CInuError  GetSensorParams(CSensorParams& oParams, ECameraName iCameraName) const = 0;

		/// \brief    Overwrite InuSensor initialized  parameters.
		/// 
		/// If this function is invoked after Start then only FPS will be changed and the new resolution will be ignored.
		/// \param[in] oParams    New parameters to apply.
		/// \param[in] iCameraName    To which camera to apply these parameters, if eAllCameras is used then it is applied to all.
		/// \return CInuError    Error code, InDev::eOK if operation successfully completed.
		virtual CInuError  SetSensorParams(const CSensorParams& iParams, ECameraName iCameraName) = 0;
        
		/// \brief    Get information about the SW and HW components. 
		/// \param[out] oVersion    Version description of each component.
		/// \return CInuError    Error code, InDev::eOK if operation successfully completed.
		virtual CInuError GetVersion(std::map<CEntityVersion::EEntitiesID, CEntityVersion>& oVersion) = 0;

		/// \brief    Get the configuration of InuSensor. 
        ///
        /// Should be called only if after the sensor was initiated. 
        /// \param[out] oConfiguration    returned configuration.
        /// \return CInuError    Error code, InDev::eOK if operation successfully completed.
        virtual CInuError GetSensorConfiguration(CSensorConfiguration& oConfiguration) const = 0;

        /// \brief    Get Sensor Control (AGC) data.
        ///
        /// It should be called only if after any Video related stream (e.g. Video, Depth, Head, etc.) was initiated. 
		/// \param[out] oParams    Sensor Control parameters.
		/// \param[in] iChannel    Which channel, it is possible to define different parameters to each channel of video stream.
		/// \return CInuError    Error code, InDev::eOK if operation successfully completed.
        virtual CInuError GetSensorControlParams(CSensorControlParams& oParams, uint32_t iChannel = 0) const = 0;

        /// \brief    Set Sensor Control (AGC) data. 
        /// 
        /// It should be called only if after any Video related stream (e.g. Video, Depth, Head, etc.) was initiated. 
        /// \param[in] iParams    New Sensor Control parameters.
		/// \param[in] iChannel    Which channel, it is possible to define different parameters to each channel of video stream.
		/// \return CInuError    Error code, InDev::eOK if operation successfully completed.
        virtual CInuError SetSensorControlParams(const CSensorControlParams& iParams, uint32_t iChannel = 0) const = 0;

        /// \brief    Load Registers from input file. 
        /// 
        /// It should be called only if after the sensor was initiated. 
        /// The file format should be provided by Inuitive technical staff.
        /// \param[in] iFileName    Input file name provided by Inuitive.
        /// \return CInuError    Error code, InDev::eOK if operation successfully completed.
        virtual CInuError LoadRegistersConfigurationFile(std::string iFileName) const = 0;


        /// \brief		Get Optical data information 
        /// \param[out]  oOpticalData   Output optical data information
        /// \return CInuError   If InuSensor isn't initialized then eStateError is returned 
        virtual CInuError GetOpticalData(COpticalData& oOpticalData) const  = 0;

        /// \brief		Set one of the assembled projectors' state
        /// 
        /// \param[in]  iLevel : High - high power, Low low power, Off - projector off 
        /// \param[in]  EProjectors - Projector name, eNumOfProjectors is illegal value
        /// \return CInuError
        virtual CInuError SetProjectorLevel(EProjectorLevel iLevel, EProjectors iProjectorID) const = 0;

        /// \brief		Get one of the assembled projectors' state
        /// 
        /// \param[out]  iLevel : High - high power, Low low power, Off - projector off 
        /// \param[out]  EProjectors - Projector name, eNumOfProjectors is illegal value
        /// \return CInuError
        virtual CInuError GetProjectorLevel(EProjectorLevel& iLevel, EProjectors iProjectorID) const = 0;


    protected:

        CInuSensor() {}
    };

}

#endif  // __INUSENSOR_H__

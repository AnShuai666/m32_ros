
#ifndef __DEPTHSTREAMEXT_H__
#define __DEPTHSTREAMEXT_H__

#include "DepthStream.h"
#include "ImageFrameExt.h"
#include "BaseStreamExt.h"

namespace InuDev
{
    struct CSensorParamsExt;

    struct CDepthInitParams : CImageFrameParams
    {
    };

    struct CDepthFilterParams 
    {
        enum InputTypes
        {
            eDepth = 0,
            eDisparityNoStretch = 1,
            eDisparityStretch = 2,
            eBasicOptions = 2,
            eLeftY = 3,
            eRightY = 4,
            eOption6 = 6,
            eOption7 = 7,
            eOption8 = 8,
            eOption9 = 9
        };

        enum ColorTypes
        {
            eColor = 0,
            eGray = 1
        };


        CDepthFilterParams() : filterType(eDepth), colorType(eColor){}
        InputTypes filterType;
        ColorTypes colorType;

    };

    ////////////////////////////////////////////////////////////////////////
    /// \brief DepthStream extension class 
    ///
    ///
    /// Role: Adds extended features to the DepthStream interface
    ///       Intended for internal use
    ///
    ///
    /// Responsibilities: 
    ///      1. Add extended API interface
    ///
    /// Comments: 
    ////////////////////////////////////////////////////////////////////////
    class CDepthStreamExt : public CDepthStream, public CBaseStreamExt
    {

    public:

        typedef std::function<void(std::shared_ptr<CDepthStreamExt> , const CImageFrameExt&,  CInuError)> CallbackFunctionExt;
        using CDepthStream::Init;
        using CDepthStream::GetFrame;

        CDepthStreamExt() {}
        virtual ~CDepthStreamExt() {}

        //Extended methods

        virtual CInuError         SetDisplayParams(const CDepthFilterParams& params) = 0;
        virtual CInuError         GetDisplayParams(CDepthFilterParams& params) = 0;

        virtual CInuError         SetParams(const CDepthInitParams& params) = 0;
        virtual CInuError         GetParams(CDepthInitParams& params) = 0;

        virtual CInuError         GetFrame(CImageFrameExt& oStream, unsigned int iTimeout = FPS_BASED_TIMEOUT, bool iCopyLocal = true) =0;

        virtual CInuError         Register(CallbackFunctionExt iCallback, unsigned int iTimeout = FPS_BASED_TIMEOUT, bool iCopyLocal = true) = 0;

        /// \brief set point cloud FPS
        /// 
        /// Detailed description:        Set point cloud FPS. (by default is set to be same as the depth frame FPS) 
        /// \param[in] iPointCoudFPS     The new fps
        /// \return InuDev::CInuError    Return error if user sets the fps to be 0 or greater than the depth fps, 
        ///                              otherwise, it is ok.
        virtual CInuError         SetPointCloudFPS(unsigned int iFPS ) = 0;

    protected:


    private:

    };
}

#endif // __DEPTHSTREAMEXT_H__


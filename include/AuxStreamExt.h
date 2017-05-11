
#ifndef __AUXSTREAMEXT_H__
#define __AUXSTREAMEXT_H__

#include "AuxStream.h"
#include "BaseStreamExt.h"
#include "BaseFrameExt.h"

namespace InuDev
{

    class CImuFrameExt : public CImuFrame    
    {

    };

    class CAuxStreamExt : public CAuxStream, public CBaseStreamExt
    {

    public:
        using CAuxStream::Init;
        using CAuxStream::GetFrame;


        /// \brief    Retrieves CImuFrame frame (pull) 
        /// 
        /// This method returns when a new CImuFrame is ready (blocking) or if an input timeout has elapsed. 
        /// It shall be called only after a Start() was is invoked and but before any invocation of a Stop() is invoked.
        /// \param[out] oAuxFrame    Array of CImuData sensor values.
        /// \param[in] iTimeout    Function is returned if timeout has elapsed even if no new frame is ready.
        /// \return CInuError    Error code, InDev::eOK if operation successfully completed.

        virtual CInuError GetFrame(CImuFrameExt& oAuxFrame, unsigned int iTimeout = FPS_BASED_TIMEOUT) = 0;

        CAuxStreamExt() {}
        virtual ~CAuxStreamExt() {}

    protected:


    private:

    };
}

#endif // __AUXSTREAMEXT_H__


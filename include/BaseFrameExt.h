/*
* File - IAFFrames.h
*
* This file is part of Inuitive API Framework  and it defines base class of all provided frames
*
*/


#ifndef __BASEFRAMEEXT_H__
#define __BASEFRAMEEXT_H__

namespace InuDev
{

    ////////////////////////////////////////////////////////////////////////
    /// \brief    Common behavior of all IAF NUI frames. 
    ///
    /// Role: Base class of all InuStreams' frames.
    ///
    /// Responsibilities: 
    ///      1. Common members. 
    ///      2. Construction is enabled only from derived classes.
    ///
    ////////////////////////////////////////////////////////////////////////
    class  CBaseFrameExt 
    {
    public:
        /// \brief    true if this frame was recorded by InuService
        bool WasRecorded; 
        
        CBaseFrameExt() : WasRecorded(false) {}
                
        virtual ~CBaseFrameExt() {}
    };
}

#endif // __BASEFRAMEEXT_H__

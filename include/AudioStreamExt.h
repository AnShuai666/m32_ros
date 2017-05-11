/*
* File - AudioStreamExt.h
*
* This file is part of the Inuitive SDK
*
* Copyright (C) 2014 All rights reserved to Inuitive
*
*/


#ifndef __AUDIOSTREAMEXT_H__  
#define __AUDIOSTREAMEXT_H__

#include "AudioStream.h"
#include "BaseStreamExt.h"

namespace InuDev
{

    class CAudioStreamExt : public CAudioStream, public CBaseStreamExt
    {

    public:
       // using CAudioStreamExt::WasRecorded=true;
        using CAudioStream::Init;
        using CAudioStream::GetFrame;

        CAudioStreamExt() { }
        virtual ~CAudioStreamExt() {}

    protected:


    private:

    };
}

#endif //__AUDIOSTREAMEXT_H__

/*  SrDoublerWmpPlg Windows Media Player Sample Rate Doubling Plugin
    Copyright (C) Lev Minkovsky
    
    This file is part of SrDoublerWmpPlg.

    SrDoublerWmpPlg is free software; you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation; either version 2 of the License, or
    (at your option) any later version.

    SrDoublerWmpPlg is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with SrDoublerWmpPlg; if not, write to the Free Software
    Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
/////////////////////////////////////////////////////////////////////////////
//
// SrDoublerWmpPlg.h : Declaration of CSrDoublerWmpPlg
//
  
#pragma once

#include "resource.h"
#include <mediaobj.h>       // The IMediaObject header from the DirectX SDK.
#include "wmpservices.h"    // The header containing the WMP interface definitions.
#include <vector>
using namespace std;

const DWORD UNITS = 10000000;  // 1 sec = 1 * UNITS

// registry location for preferences
const TCHAR kszPrefsRegKey[] = _T("Software\\SrDoublerWmpPlg\\DSP Plugin");

// {59D164DD-FA65-4A7F-A700-85582EC905D2}
DEFINE_GUID(CLSID_SrDoublerWmpPlg, 0x59d164dd, 0xfa65, 0x4a7f, 0xa7, 0x0, 0x85, 0x58, 0x2e, 0xc9, 0x5, 0xd2);

inline int RaiseEx(const char * message)
{
   static ULONG_PTR ptr_array[1];
   ptr_array[0]=reinterpret_cast<const unsigned long>(message);
   RaiseException(1,EXCEPTION_NONCONTINUABLE,1,ptr_array);
   return 0;//we well never get here
}

//An interleaved audio stream buffer accessible as an array of frames
class CFrameBuffer
{
   vector<char> m_buf;
   int m_nSampleSize;
   int m_nFrameSize;
   int m_nChannels;
   int m_nSampleRate; 
   bool m_bFloat;//this is needed to distinguish between float and 32-bit PCM data
public:
   CFrameBuffer():m_nSampleSize(2),m_nChannels(2),m_nSampleRate(0),m_bFloat(false)
   {
      m_nFrameSize=m_nSampleSize*m_nChannels;
   }
   int getChannels() const {return m_nChannels;}
   int getSampleRate() const {return m_nSampleRate;}
   int getFrameSize() const {return m_nFrameSize;}
   int getSampleSize() const {return m_nSampleSize;}
   bool isFloat() const {return m_bFloat;}
   
   void init(int sampleSize,int nChannels,int sampleRate,bool bFloat)
   {
      m_nSampleRate=sampleRate;
      m_nSampleSize=sampleSize;
      m_nChannels=nChannels;
     if (m_nSampleSize==0)
        RaiseEx("m_nSampleSize is zero");
     if (m_nChannels==0)
        RaiseEx("nChannels is zero");
      m_nFrameSize=m_nSampleSize*m_nChannels;
      m_bFloat=bFloat;
   }

   char&  operator[](int i)
   {
      if (m_buf.size()==0)
         RaiseEx("Frame buffer has not been allocated");
      return *(&m_buf[0]+m_nFrameSize*i);
   }
   const char& operator[](int i) const
   {
      if (m_buf.size()==0)
         RaiseEx("Frame buffer has not been allocated");
      return *(&m_buf[0]+m_nFrameSize*i);
   }
   int getSize() const 
   {
      if (!m_nFrameSize)
         RaiseEx("m_nFrameSize is 0");
      return m_buf.size()/m_nFrameSize;
    }
   void reserve(int nCapacity) 
   {
      m_buf.reserve(nCapacity*m_nFrameSize);
   }
   void append(const char * pFrames, int nNumFrames)
   {
      m_buf.insert(m_buf.end(),pFrames,pFrames+nNumFrames*m_nFrameSize);
   }
   void truncate_beginning(unsigned nNumFrames)
   {
      if (m_buf.size()<nNumFrames)
         RaiseEx("Illegal trancate");
      m_buf.erase(m_buf.begin(),m_buf.begin()+nNumFrames*m_nFrameSize);
   }
   void expand(int nNumFrames)
   {
      m_buf.resize(m_buf.size()+nNumFrames*m_nFrameSize);
   }
   void truncate_end(unsigned nNumFrames)
   {
      if (m_buf.size()<nNumFrames)
         RaiseEx("Illegal trancate");
      m_buf.resize(m_buf.size()-nNumFrames*m_nFrameSize);
   }
   void clear()
   {
      m_buf.clear();
   }
};

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg
/////////////////////////////////////////////////////////////////////////////

class ATL_NO_VTABLE CSrDoublerWmpPlg : 
    public CComObjectRootEx<CComMultiThreadModel>,
    public CComCoClass<CSrDoublerWmpPlg, &CLSID_SrDoublerWmpPlg>,
    public IMediaObject,
    public IWMPPluginEnable
{
public:
    CSrDoublerWmpPlg();
    virtual ~CSrDoublerWmpPlg();

DECLARE_REGISTRY_RESOURCEID(IDR_SRDOUBLERWMPPLG)

DECLARE_PROTECT_FINAL_CONSTRUCT()

BEGIN_COM_MAP(CSrDoublerWmpPlg)
    COM_INTERFACE_ENTRY(IMediaObject)
    COM_INTERFACE_ENTRY(IWMPPluginEnable)
END_COM_MAP()

    // CComCoClass Overrides
    HRESULT FinalConstruct();
    void    FinalRelease();


    // IMediaObject methods
    STDMETHOD( GetStreamCount )( 
                   DWORD *pcInputStreams,
                   DWORD *pcOutputStreams
                   );
    
    STDMETHOD( GetInputStreamInfo )( 
                   DWORD dwInputStreamIndex,
                   DWORD *pdwFlags
                   );
    
    STDMETHOD( GetOutputStreamInfo )( 
                   DWORD dwOutputStreamIndex,
                   DWORD *pdwFlags
                   );
    
    STDMETHOD( GetInputType )( 
                   DWORD dwInputStreamIndex,
                   DWORD dwTypeIndex,
                   DMO_MEDIA_TYPE *pmt
                   );
    
    STDMETHOD( GetOutputType )( 
                   DWORD dwOutputStreamIndex,
                   DWORD dwTypeIndex,
                   DMO_MEDIA_TYPE *pmt
                   );
    
    STDMETHOD( SetInputType )( 
                   DWORD dwInputStreamIndex,
                   const DMO_MEDIA_TYPE *pmt,
                   DWORD dwFlags
                   );
    
    STDMETHOD( SetOutputType )( 
                   DWORD dwOutputStreamIndex,
                   const DMO_MEDIA_TYPE *pmt,
                   DWORD dwFlags
                   );
    
    STDMETHOD( GetInputCurrentType )( 
                   DWORD dwInputStreamIndex,
                   DMO_MEDIA_TYPE *pmt
                   );
    
    STDMETHOD( GetOutputCurrentType )( 
                   DWORD dwOutputStreamIndex,
                   DMO_MEDIA_TYPE *pmt
                   );
    
    STDMETHOD( GetInputSizeInfo )( 
                   DWORD dwInputStreamIndex,
                   DWORD *pcbSize,
                   DWORD *pcbMaxLookahead,
                   DWORD *pcbAlignment
                   );
    
    STDMETHOD( GetOutputSizeInfo )( 
                   DWORD dwOutputStreamIndex,
                   DWORD *pcbSize,
                   DWORD *pcbAlignment
                   );
    
    STDMETHOD( GetInputMaxLatency )( 
                   DWORD dwInputStreamIndex,
                   REFERENCE_TIME *prtMaxLatency
                   );
    
    STDMETHOD( SetInputMaxLatency )( 
                   DWORD dwInputStreamIndex,
                   REFERENCE_TIME rtMaxLatency
                   );
    
    STDMETHOD( Flush )( void );
    
    STDMETHOD( Discontinuity )( 
                   DWORD dwInputStreamIndex
                   );
    
    STDMETHOD( AllocateStreamingResources )( void );
    
    STDMETHOD( FreeStreamingResources )( void );
    
    STDMETHOD( GetInputStatus )( 
                   DWORD dwInputStreamIndex,
                   DWORD *pdwFlags
                   );
    
    STDMETHOD( ProcessInput )( 
                   DWORD dwInputStreamIndex,
                   IMediaBuffer *pBuffer,
                   DWORD dwFlags,
                   REFERENCE_TIME rtTimestamp,
                   REFERENCE_TIME rtTimelength
                   );
    
    STDMETHOD( ProcessOutput )( 
                   DWORD dwFlags,
                   DWORD cOutputBufferCount,
                   DMO_OUTPUT_DATA_BUFFER *pOutputBuffers,
                   DWORD *pdwStatus
                   );

    STDMETHOD( Lock )( LONG bLock );

    // Note: need to override CComObjectRootEx::Lock to avoid
    // ambiguity with IMediaObject::Lock. The override just
    // calls through to the base class implementation.

    // CComObjectRootEx overrides
    void Lock()
    {
        CComObjectRootEx<CComMultiThreadModel>::Lock();
    }

    // IWMPPluginEnable methods
    STDMETHOD( SetEnable )( BOOL fEnable );
    STDMETHOD( GetEnable )( BOOL *pfEnable );

private:
    HRESULT ValidateInputMediaType(
                const DMO_MEDIA_TYPE *pmtTarget);    // target media type to verify
    HRESULT ValidateOutputMediaType(
                const DMO_MEDIA_TYPE *pmtTarget);    // target media type to verify
                
    void doubleSampleRate();                     //does the core processing

    DMO_MEDIA_TYPE          m_mtInput;          // Stores the input format structure
    DMO_MEDIA_TYPE          m_mtOutput;         // Stores the output format structure

    bool                    m_bValidTime;       // Is timestamp valid?
    REFERENCE_TIME          m_rtTimestamp;      // Stores the input buffer timestamp

    BOOL                    m_bEnabled;         // TRUE if enabled
    bool                    m_bDiscontinuityCalled;
    CFrameBuffer            m_inputBuffer;
    CFrameBuffer            m_outputBuffer;
};


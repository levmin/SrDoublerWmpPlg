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
// SRDoublerWmpPlg.cpp 
//
/////////////////////////////////////////////////////////////////////////////

#include "stdafx.h"
#include "SRDoublerWmpPlg.h"

#include <mediaerr.h>   // DirectX SDK media errors
#include <dmort.h>      // DirectX SDK DMO runtime support
#include <uuids.h>      // DirectX SDK media types and subtyes
#include <ks.h>         // required for WAVEFORMATEXTENSIBLE
#include <Mmreg.h>
#include <ksmedia.h>
#include <stdio.h>
#define _USE_MATH_DEFINES
#include <math.h>
#include <vector>
#include <float.h>
#include <gsl\span>
using namespace std;
#include <emmintrin.h>

#define DUMP_OUTPUT FALSE

#if DUMP_OUTPUT 
#include "sndfile.h"
#endif

#define RETURN_UNLESS(value,retValue)  if(!(value)) return retValue

const int TABLE_WIDTH=3200;         //width of a filter table

//detect if we run on the CPU that supports SSE2 instructions
inline bool IsSSE2Present()
{
   unsigned dwFeature=0;
   __try {
        _asm {
         mov eax, 1
         cpuid
         mov dwFeature, edx
        }
    }
    __except (1) {//this is the value of EXCEPTION_EXECUTE_HANDLER defined in excpt.h
        return false;
    }
    return !!(dwFeature & 0x04000000);
}

const bool s_bIsSSE2Present=IsSSE2Present();

// zero-th order modified Bessel function of the first kind
inline double I0(double z)
{
   double zz4 = z*z / 4.;
   double EPS = 10E-16;
   double k = 0.;       //summing by k from 0 to infinity
   double zz4_in_k_degree = 1.;
   double kfact = 1.;   //k factorial
   double I0_sum_member = 1.; //the sum member for k = 0
   double I0_sum = I0_sum_member; 
   do
   {
      k++; //increment k
      kfact *= k; //calculate k factorial
      zz4_in_k_degree *= zz4;
      I0_sum_member = zz4_in_k_degree / (kfact*kfact);
      I0_sum += I0_sum_member;
   } while (I0_sum_member >= EPS);
   return I0_sum;
}


//Keiser window function for a floating point argument
inline double Kaiser(double x,double alpha)
{
   if (x < 0.)
      return 1.;
   else if (x > 1.)
      return 0.;
   else
      return I0(alpha*sqrt(1. - x*x)) / I0(alpha);
}

//A standard Keiser function goes from 1 to 0 when its argument goes from 0 to 1
//The mapped function does this when its argument goes from n0 to n1 
inline double KaiserMappedOverIntegerRange(double x,double alpha,int n0, int n1)
{
   if (n0!=n1)
      return Kaiser((x-n0)/(n1-n0),alpha);
   else
      return RaiseEx("Wrong KaiserMappedOverIntegerRange params");
}

inline double sinc(double x)
{
   if (x==0.)
      return 1.;
   else 
      return sin(M_PI*x)/(M_PI*x);
}

/* Keiser window filter
   The class allocates a 16-byte aligned double array and fills
   it with the values of a Keiser window function mapped over the range
   from 0 to halfWidth+1, for arguments from 0.5 to halfWidth-0.5,
   multiplied by the values of a sinc function for the same arguments.
*/
class CFilter
{
public:
   CFilter(size_t halfWidth,double alpha);
   ~CFilter();
   const double * getTable() const {return m_table;}
   const int getHalfSize() const  {return m_halfWidth;}
private:
   double * m_table;
   int m_halfWidth;
};

CFilter::CFilter(size_t halfWidth,double alpha):m_table(NULL),m_halfWidth(halfWidth)
{
   if (halfWidth <=0)
      RaiseEx("Wrong halfWidth");
   //allocate the filter table
   m_table=static_cast<double*>(_aligned_malloc(2*halfWidth*sizeof(double),16));
   if (!m_table)
      RaiseEx("m_table allocation failure");
   //calculate the values
   typedef gsl::span<double> double_span;
   double_span table(m_table, 2 * halfWidth);
   double_span::iterator middle = table.begin() + halfWidth;

   size_t i = 0;
   
   auto lambda = [&] 
      {
         size_t dist = (i < halfWidth) ? (halfWidth - i - 1) : (i - halfWidth);
         i++;
         return KaiserMappedOverIntegerRange(dist+0.5, alpha, 0, halfWidth+1)*sinc(dist+0.5);
      };
   
   generate(table.begin(), table.end(), lambda );
}

CFilter::~CFilter()
{ 
    _aligned_free(m_table);
} 

//this class scrolls the Keiser filter along the input stream
//and calculates interpolated samples
class CScrollBuffer
{
   public:
       
       CScrollBuffer(const CFrameBuffer& input);
       ~CScrollBuffer(){}
       
       double getCurrentSample(int nChannel) const
         {
         return bufferSample(nChannel,TABLE_WIDTH/2-1);
         }

       int getNumFrames() const 
         {
         return m_input.getSize();
         }

       bool scroll();    

       double getInterpolatedSample (int channel) const;

private:

       void getNextSamples(int offset);

       /* All buffer sample operations are done through this function.
          It provides an address translation from the filter area
          of the sample buffer to the beginning of the buffer.
       */
       double& bufferSample(int channel, int offset) const
         {
         return (double&)m_sampleBuffer[channel][m_nFilterPos+offset];
         }

       /*Convert the sample stored in lower bits of a integer parameter
         into a valid integer value by removing the extra upper bits and 
         promoting a sign bit.
       */
       void promoteSignBit(int & value) const
         {
         int offset=(sizeof(int)-m_nSampleSize)*8;
         value<<=offset;
         value>>=offset;
         }
       
       const CFrameBuffer& m_input;
       int   m_nLastSampleNumber;
       unsigned m_nSampleSize;//in bytes
       unsigned m_nMask;  //masks m_nSampleSize least significant bytes as 1s and the rest as 0s
       double m_dScaleFactor; //we want our doubles to be in the [-1,1] range, so scaling is necessary
       int      m_nChannels;
       vector< vector<double> >  m_sampleBuffer;
       int      m_nFilterPos; //current filter position in the m_sampleBuffer

       static const int ALPHA=9;                  //parameter of a Kaiser function
       static CFilter m_KeiserFilter;
};

CFilter CScrollBuffer::m_KeiserFilter(TABLE_WIDTH/2,ALPHA);

CScrollBuffer::CScrollBuffer(const CFrameBuffer& input) 
    :m_input(input),m_nFilterPos(0)
{
   m_nSampleSize=input.getFrameSize()/input.getChannels();
   if (m_nSampleSize>4 || m_nSampleSize<2)
      RaiseEx("Unexpected sample size");
   
   const static double scaleFactors[]={1<<15,1<<23,((unsigned)1)<<31};
   //lookup the scale factor from the table
   m_dScaleFactor=scaleFactors[m_nSampleSize-2];
   
   m_nChannels = input.getChannels();
   m_sampleBuffer.resize(m_nChannels);
   
   //every channel should have a sample buffer with the size 2*TABLE_WIDTH-1
   for (int channel=0;channel<m_nChannels;channel++)
      m_sampleBuffer[channel].resize(2*TABLE_WIDTH-1);
   
   m_nLastSampleNumber=-1;
   //masks m_nSampleSize least significant bytes as 1s and the rest as 0s
   m_nMask=       ((unsigned)(0xFFFFFFFF))>>((sizeof(unsigned)-m_nSampleSize)*8);
   //fill the buffer with samples
   for (int i=0;i<TABLE_WIDTH;i++)
      getNextSamples(i);
}

/* Copies the next set of input samples from all channels into 
   elements of the sample buffer with an index m_nFilterPos+offset
*/
void CScrollBuffer::getNextSamples(int offset)
{
    m_nLastSampleNumber++;
    if (m_nLastSampleNumber>=m_input.getSize())
        RaiseEx("Past segment boundary");
    const char * raw_frame=&m_input[m_nLastSampleNumber];
    //copy input samples from the raw_frame into m_sampleBuffer converting them to double
    if (m_input.isFloat())
    { 
        for (int channel=0;channel<m_nChannels;channel++)
            bufferSample(channel,offset)=(double)(*(float*)(raw_frame+m_nSampleSize*channel));
    }
    else
    {   //convert input samples into doubles
        for (int channel=0;channel<m_nChannels;channel++)
        {    
            int sample=(*(unsigned*)(raw_frame+m_nSampleSize*channel)) & m_nMask;
            promoteSignBit(sample);
            //we want our doubles to be in the [-1,1] range, so scaling is necessary
            bufferSample(channel,offset)=sample/m_dScaleFactor;
        }
    }    
}

/*  Increment or wrap a filter position in the buffer and
    get new samples from the input buffer.
*/
bool CScrollBuffer::scroll() 
{
   if (m_nLastSampleNumber>=m_input.getSize()-1)
      return false;
/* If the filter is at the end of the buffer 
        then move it to the beginning of the buffer;
        otherwise increment it's position in the buffer.
*/
   m_nFilterPos =(m_nFilterPos==TABLE_WIDTH-1)?0:m_nFilterPos+1;
   
/* Get next set of input samples into last elements of the filter area.
   If the filter is not at the beginning of the buffer
        copy these new samples into a elements right before the filter.  
*/
   getNextSamples(TABLE_WIDTH-1);
   if (m_nFilterPos > 0)
       for (int channel=0;channel<m_nChannels;channel++)
           bufferSample(channel,-1)=bufferSample(channel,TABLE_WIDTH-1);
   return true;
}

/*  Compute and return an interpolated sample value.
*/
double CScrollBuffer::getInterpolatedSample(int channel) const
{
   const int tableSize=m_KeiserFilter.getHalfSize()*2;
   const double * fpBufPtr=&bufferSample(channel,0);
   const double * filterPtr=m_KeiserFilter.getTable();
   if (::s_bIsSSE2Present)
   {
      double result;
/*
   The inline assembler is an optimized rendition
   of the following code:

   __m128d sum=_mm_setzero_pd();
   for (int i=0;i<tableSize;i+=2)
   {
      __m128d fpBufPair=_mm_loadu_pd(&fpBufPtr[i]);
      __m128d filterPair=_mm_load_pd(&filterPtr[i]);
      __m128d mult=_mm_mul_pd(fpBufPair,filterPair);
      sum=_mm_add_pd(sum,mult);
   }
   double result[2];
   _mm_storeu_pd(result,sum);
   return result[0]+result[1];
*/
      __asm
      {
        mov       eax, DWORD PTR[fpBufPtr]
        mov       edx, DWORD PTR[filterPtr]
        mov       ecx, tableSize

        pxor      xmm0, xmm0                                    
        test      ecx, ecx                                      
        jle       $END_OF_LOOP
                                
        xor       esi, esi                                      
                                
$LOOP:                     
        movsd     xmm1, QWORD PTR [eax+esi*8]                   
        movhpd    xmm1, QWORD PTR [eax+esi*8+8]                 
        mulpd     xmm1, XMMWORD PTR [edx+esi*8]                 
        addpd     xmm0, xmm1                                    
        add       esi, 2                                        
        cmp       esi, ecx                                      
        jl        $LOOP
                                
$END_OF_LOOP:                         

        movhpd    result, xmm0                 
        addsd     xmm0, result                   

        movsd     result, xmm0 
      }

      return result;
   }
   else
   {
      double result=0;
      for (int i=0;i<tableSize;i++)
      {
         result+=bufferSample(channel,i)*m_KeiserFilter.getTable()[i];
      }
      return result;
   }
}

template<typename SampleFormat, uint8_t numChannels> class SRDoubler
{
public:
   typedef std::array<SampleFormat,numChannels> SampleFrame;
   typedef gsl::span<SampleFrame> FrameSpan;
private:
   FrameSpan m_in_buffer;
   FrameSpan m_out_buffer;
   vector<FrameSpan> m_prefixBuffer;
   
   const FrameSpan& getInputFrame(size_t num)
   {
      return (num >= m_prefixBuffer.size()) : m_in_buffer[num - m_prefixBuffer.size()] : m_prefixBuffer[num];
   }

   void Internal_Run()
   {

   }

public:

   void SetInputBuffer (FrameSpan&& in_span)  
   { 
      m_in_buffer = in_span; 
   }
   void SetOutputBuffer(FrameSpan&& out_span) 
   { 
      m_out_buffer = out_span; 
   }
   SRDoubler(size_t table_size) : m_prefixBuffer(table_size,0)
   {
   }
   
   void Run()
   {
      if (m_prefixBuffer.size() > m_in_buffer.size())
      {
         auto border = m_prefixBuffer.begin() + (m_prefixBuffer.size() - m_in_buffer.size());
         //zero out the beginning of the array
         std::for_each(m_prefixBuffer.begin(), border, [](SampleFrame& value) {value.assign(0); });
         std::copy(m_in_buffer.begin(), m_in_buffer.end(), border);
      }
      else
      {
         std::copy(m_in_buffer.begin(), m_in_buffer.begin() + m_prefixBuffer.size() , m_prefixBuffer.begin());
      }
   }
};

/*
       The very plug-in
*/

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::CSrDoublerWmpPlg
//
// Constructor
/////////////////////////////////////////////////////////////////////////////
CSrDoublerWmpPlg::CSrDoublerWmpPlg()
{
    m_bValidTime = false;
    m_rtTimestamp = 0;
    m_bEnabled = TRUE;
    m_bDiscontinuityCalled=false;

    ::ZeroMemory(&m_mtInput, sizeof(m_mtInput));
    ::ZeroMemory(&m_mtOutput, sizeof(m_mtOutput));
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::~CSrDoublerWmpPlg
//
// Destructor
/////////////////////////////////////////////////////////////////////////////

CSrDoublerWmpPlg::~CSrDoublerWmpPlg()
{
    ::MoFreeMediaType(&m_mtInput);
    ::MoFreeMediaType(&m_mtOutput);
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::FinalConstruct
//
// Called when an plug-in is first loaded. Use this function to do one-time
// intializations that could fail instead of doing this in the constructor,
// which cannot return an error.
/////////////////////////////////////////////////////////////////////////////

HRESULT CSrDoublerWmpPlg::FinalConstruct()
{
    return S_OK;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg:::FinalRelease
//
// Called when an plug-in is unloaded. Use this function to free any
// resources allocated.
/////////////////////////////////////////////////////////////////////////////

void CSrDoublerWmpPlg::FinalRelease()
{
    FreeStreamingResources();  // In case client does not call this.        
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::GetStreamCount
//
// Implementation of IMediaObject::GetStreamCount
/////////////////////////////////////////////////////////////////////////////

STDMETHODIMP CSrDoublerWmpPlg::GetStreamCount( 
               DWORD *pcInputStreams,
               DWORD *pcOutputStreams)
{
    HRESULT hr = S_OK;

    RETURN_UNLESS(pcInputStreams,E_POINTER);
    RETURN_UNLESS(pcOutputStreams,E_POINTER);

    // The plug-in uses one stream in each direction.
    *pcInputStreams = 1;
    *pcOutputStreams = 1;

    return S_OK;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::GetInputStreamInfo
//
// Implementation of IMediaObject::GetInputStreamInfo
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::GetInputStreamInfo( 
               DWORD dwInputStreamIndex,
               DWORD *pdwFlags)
{    
    RETURN_UNLESS(pdwFlags,E_POINTER);

    // The stream index must be zero.
    RETURN_UNLESS(dwInputStreamIndex==0, DMO_E_INVALIDSTREAMINDEX);

    // Use the default input stream configuration (a single stream). 
    *pdwFlags = DMO_INPUT_STREAMF_WHOLE_SAMPLES ;

    return S_OK;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::GetOutputStreamInfo
//
// Implementation of IMediaObject::GetOutputStreamInfo
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::GetOutputStreamInfo( 
               DWORD dwOutputStreamIndex,
               DWORD *pdwFlags)
{
    RETURN_UNLESS(pdwFlags,E_POINTER);

    // The stream index must be zero.
    RETURN_UNLESS(dwOutputStreamIndex==0, DMO_E_INVALIDSTREAMINDEX);

    // Use the default output stream configuration (a single stream).
    *pdwFlags = 0;

    return S_OK;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::GetInputType
//
// Implementation of IMediaObject::GetInputType
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::GetInputType ( 
               DWORD dwInputStreamIndex,
               DWORD dwTypeIndex,
               DMO_MEDIA_TYPE *pmt)
{
   RETURN_UNLESS(m_bEnabled,S_FALSE);
   RETURN_UNLESS(dwInputStreamIndex==0, DMO_E_INVALIDSTREAMINDEX);
    // only support one preferred type
    RETURN_UNLESS(dwTypeIndex==0,DMO_E_NO_MORE_ITEMS);
    RETURN_UNLESS(pmt,E_POINTER);

   if (m_mtInput.majortype!=GUID_NULL)
     return GetInputCurrentType(dwInputStreamIndex,pmt);

   ::ZeroMemory( pmt, sizeof( DMO_MEDIA_TYPE ) );
   pmt->majortype = MEDIATYPE_Audio;
   pmt->subtype = MEDIASUBTYPE_PCM;
   return S_OK;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::GetOutputType
//
// Implementation of IMediaObject::GetOutputType
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::GetOutputType( 
               DWORD dwOutputStreamIndex,
               DWORD dwTypeIndex,
               DMO_MEDIA_TYPE *pmt)
{
   RETURN_UNLESS(m_bEnabled,S_FALSE);
   HRESULT hr = S_OK;

   RETURN_UNLESS(dwOutputStreamIndex==0,DMO_E_INVALIDSTREAMINDEX);
   // only support one preferred type
   RETURN_UNLESS(dwTypeIndex==0,DMO_E_NO_MORE_ITEMS);
   RETURN_UNLESS(pmt,E_POINTER);

   if (m_mtOutput.majortype!=GUID_NULL)
      return GetOutputCurrentType(dwOutputStreamIndex,pmt);

   ::ZeroMemory( pmt, sizeof( DMO_MEDIA_TYPE ) );
   if (m_mtInput.pbFormat)
   {
      //We output data in the WAVEFORMATEXTENSIBLE format with a subformat KSDATAFORMAT_SUBTYPE_IEEE_FLOAT
     hr = ::MoInitMediaType(pmt,sizeof(WAVEFORMATEXTENSIBLE));
     if (FAILED(hr))
       return hr;
     const int nChannels=((WAVEFORMATEX*)m_mtInput.pbFormat)->nChannels;
      const int wBitsPerSample=32;
     const int nBlockAlign=nChannels*wBitsPerSample/8;
     //we double the nSamplesPerSec of the input format
      const int nSamplesPerSec=((WAVEFORMATEX*)m_mtInput.pbFormat)->nSamplesPerSec*2;
     const int nAvgBytesPerSec=nSamplesPerSec*nBlockAlign;
     pmt->majortype = m_mtInput.majortype;
      pmt->subtype = MEDIASUBTYPE_IEEE_FLOAT;
       pmt->bFixedSizeSamples=TRUE;
      pmt->bTemporalCompression=FALSE;
     pmt->lSampleSize=wBitsPerSample/8;
     pmt->formattype=FORMAT_WaveFormatEx;
     pmt->cbFormat=sizeof(WAVEFORMATEXTENSIBLE);
      
     WAVEFORMATEX * pWave=(WAVEFORMATEX * )pmt->pbFormat;
     WAVEFORMATEXTENSIBLE * pWaveXT=(WAVEFORMATEXTENSIBLE* )pmt->pbFormat;
     pWave->wFormatTag=WAVE_FORMAT_EXTENSIBLE;
     pWave->nChannels=nChannels;
     pWave->nSamplesPerSec=nSamplesPerSec;
     pWave->nAvgBytesPerSec=nAvgBytesPerSec;
     pWave->wBitsPerSample=wBitsPerSample; 
     pWave->nBlockAlign=nBlockAlign;
     pWave->cbSize=sizeof(WAVEFORMATEXTENSIBLE)-sizeof(WAVEFORMATEX);
     pWaveXT->Samples.wValidBitsPerSample=pWave->wBitsPerSample;
     if (((WAVEFORMATEX*)m_mtInput.pbFormat)->wFormatTag==WAVE_FORMAT_EXTENSIBLE)
      {// use the channel mask  of the input stream
       pWaveXT->dwChannelMask=((WAVEFORMATEXTENSIBLE*)m_mtInput.pbFormat)->dwChannelMask;
      }
      else 
      {// create a reasonable channel mask
       pWaveXT->dwChannelMask=0;
         DWORD bit=1;
         for (int i=0;i<nChannels;i++)
         {
             pWaveXT->dwChannelMask |= bit;
             bit <<= 1;
         }
      }
     pWaveXT->SubFormat=KSDATAFORMAT_SUBTYPE_IEEE_FLOAT;
   }
   else
   {
     pmt->majortype = MEDIATYPE_Audio;
     pmt->subtype = MEDIASUBTYPE_IEEE_FLOAT;
   }
   return hr;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::SetInputType
//
// Implementation of IMediaObject::SetInputType
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::SetInputType( 
               DWORD dwInputStreamIndex,
               const DMO_MEDIA_TYPE *pmt,
               DWORD dwFlags)
{
    RETURN_UNLESS(m_bEnabled,S_FALSE);
   RETURN_UNLESS(dwInputStreamIndex==0,DMO_E_INVALIDSTREAMINDEX);
   if (dwFlags&DMO_SET_TYPEF_CLEAR) 
   {//clear and return
      ::MoFreeMediaType(&m_mtInput);
      ::ZeroMemory(&m_mtInput, sizeof(m_mtInput));
      return S_OK;
   }

   RETURN_UNLESS(pmt,E_POINTER);

   HRESULT hr = ValidateInputMediaType(pmt);

   if(dwFlags & DMO_SET_TYPEF_TEST_ONLY)
      return (SUCCEEDED(hr))?S_OK:S_FALSE;  
    
   if (SUCCEEDED(hr))
   {
      // free existing media type
      ::MoFreeMediaType(&m_mtInput);
      ::ZeroMemory(&m_mtInput, sizeof(m_mtInput));

      // copy new media type
      hr = ::MoCopyMediaType( &m_mtInput, pmt );
   }
    return hr;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::SetOutputType
//
// Implementation of IMediaObject::SetOutputType
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::SetOutputType( 
               DWORD dwOutputStreamIndex,
               const DMO_MEDIA_TYPE *pmt,
               DWORD dwFlags)
{ 
   RETURN_UNLESS(m_bEnabled,S_FALSE);
   RETURN_UNLESS(dwOutputStreamIndex==0,DMO_E_INVALIDSTREAMINDEX);
   if(dwFlags & DMO_SET_TYPEF_CLEAR)
   {
     ::MoFreeMediaType( &m_mtOutput );
     ::ZeroMemory(&m_mtOutput, sizeof(m_mtOutput));
     return S_OK;
   }

   RETURN_UNLESS(pmt,E_POINTER);

   HRESULT hr = ValidateOutputMediaType(pmt);

   if (dwFlags & DMO_SET_TYPEF_TEST_ONLY)
     return (SUCCEEDED(hr))?S_OK:S_FALSE;  

   if (SUCCEEDED(hr))
   {
     //make sure the input type is properly set 
     RETURN_UNLESS(m_mtInput.majortype==MEDIATYPE_Audio,DMO_E_TYPE_NOT_ACCEPTED);
      
     // free existing media type
     ::MoFreeMediaType(&m_mtOutput);
     ::ZeroMemory(&m_mtOutput, sizeof(m_mtOutput));

     // copy new media type
     hr = ::MoCopyMediaType( &m_mtOutput, pmt );
   }
   return hr;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::GetInputCurrentType
//
// Implementation of IMediaObject::GetInputCurrentType
/////////////////////////////////////////////////////////////////////////////

STDMETHODIMP CSrDoublerWmpPlg::GetInputCurrentType( 
               DWORD dwInputStreamIndex,
               DMO_MEDIA_TYPE *pmt)
{
    RETURN_UNLESS(m_bEnabled,S_FALSE);
    RETURN_UNLESS(dwInputStreamIndex==0, DMO_E_INVALIDSTREAMINDEX);
    RETURN_UNLESS(pmt,E_POINTER);
    RETURN_UNLESS(m_mtInput.majortype!=GUID_NULL,DMO_E_TYPE_NOT_SET);

    HRESULT hr=::MoCopyMediaType( pmt, &m_mtInput );

    return hr;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::GetOutputCurrentType
//
// Implementation of IMediaObject::GetOutputCurrentType
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::GetOutputCurrentType( 
               DWORD dwOutputStreamIndex,
               DMO_MEDIA_TYPE *pmt)
{
    RETURN_UNLESS(m_bEnabled,S_FALSE);
    RETURN_UNLESS(dwOutputStreamIndex==0, DMO_E_INVALIDSTREAMINDEX);
    RETURN_UNLESS(pmt,E_POINTER);
    RETURN_UNLESS(m_mtOutput.majortype!=GUID_NULL,DMO_E_TYPE_NOT_SET);

    HRESULT hr = ::MoCopyMediaType( pmt, &m_mtOutput );

    return hr;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::GetInputSizeInfo
//
// Implementation of IMediaObject::GetInputSizeInfo
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::GetInputSizeInfo( 
               DWORD dwInputStreamIndex,
               DWORD *pcbSize,
               DWORD *pcbMaxLookahead,
               DWORD *pcbAlignment)
{
    RETURN_UNLESS(m_bEnabled,S_FALSE);
    RETURN_UNLESS(dwInputStreamIndex==0, DMO_E_INVALIDSTREAMINDEX);
    RETURN_UNLESS(pcbSize,E_POINTER);
    RETURN_UNLESS(pcbMaxLookahead,E_POINTER);
    RETURN_UNLESS(pcbAlignment,E_POINTER);
    RETURN_UNLESS(m_mtInput.majortype!=GUID_NULL,DMO_E_TYPE_NOT_SET);

   // This plug-in doesn't perform lookahead. Return zero.
   *pcbMaxLookahead = 0;

   // Get the pointer to the input format structure.
   WAVEFORMATEX *pWave = ( WAVEFORMATEX * ) m_mtInput.pbFormat;

   // Return the input sample size, in bytes.
    // We need at least TABLE_WIDTH samples for every channel
   *pcbSize = m_mtInput.lSampleSize*pWave->nChannels*TABLE_WIDTH;

   // Return the input buffer alignment, in bytes.
   *pcbAlignment = pWave->nBlockAlign;
    return S_OK;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::GetOutputSizeInfo
//
// Implementation of IMediaObject::GetOutputSizeInfo
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::GetOutputSizeInfo( 
               DWORD dwOutputStreamIndex,
               DWORD *pcbSize,
               DWORD *pcbAlignment)
{
    RETURN_UNLESS(m_bEnabled,S_FALSE);
    RETURN_UNLESS(dwOutputStreamIndex==0, DMO_E_INVALIDSTREAMINDEX);
    RETURN_UNLESS(pcbSize,E_POINTER);
    RETURN_UNLESS(pcbAlignment,E_POINTER);
    RETURN_UNLESS(m_mtOutput.majortype!=GUID_NULL,DMO_E_TYPE_NOT_SET);

    // Return the output sample size, in bytes.
    *pcbSize = m_mtOutput.lSampleSize;

    // Get the pointer to the output format structure.
    WAVEFORMATEX *pWave = ( WAVEFORMATEX * ) m_mtOutput.pbFormat;

    // Return the output buffer aligment, in bytes.
    *pcbAlignment = pWave->nBlockAlign;

    return S_OK;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::GetInputMaxLatency
//
// Implementation of IMediaObject::GetInputMaxLatency
/////////////////////////////////////////////////////////////////////////////
   
STDMETHODIMP CSrDoublerWmpPlg::GetInputMaxLatency( 
               DWORD dwInputStreamIndex,
               REFERENCE_TIME *prtMaxLatency)
{
    return E_NOTIMPL; // Not dealing with latency in this plug-in.
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::SetInputMaxLatency
//
// Implementation of IMediaObject::SetInputMaxLatency
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::SetInputMaxLatency( 
               DWORD dwInputStreamIndex,
               REFERENCE_TIME rtMaxLatency)
{
    return E_NOTIMPL; // Not dealing with latency in this plug-in.
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::Flush
//
// Implementation of IMediaObject::Flush
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::Flush( void )
{
   m_bValidTime = false;
   m_rtTimestamp = 0;
   m_bDiscontinuityCalled=false;
   m_inputBuffer.clear();
   m_outputBuffer.clear();
   return S_OK;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::Discontinuity
//
// Implementation of IMediaObject::Discontinuity
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::Discontinuity( 
               DWORD dwInputStreamIndex)
{
   if (m_inputBuffer.getSize())
      m_bDiscontinuityCalled=true;
   return S_OK;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::AllocateStreamingResources
//
// Implementation of IMediaObject::AllocateStreamingResources
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::AllocateStreamingResources ( void )
{
    return S_OK;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::FreeStreamingResources
//
// Implementation of IMediaObject::FreeStreamingResources
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::FreeStreamingResources( void )
{
    m_bValidTime = false;
    m_rtTimestamp = 0;
    m_bDiscontinuityCalled=false;
    return S_OK;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::GetInputStatus
//
// Implementation of IMediaObject::GetInputStatus
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::GetInputStatus( 
           DWORD dwInputStreamIndex,
           DWORD *pdwFlags)
{ 
    RETURN_UNLESS(m_bEnabled,S_FALSE);
    RETURN_UNLESS(dwInputStreamIndex==0, DMO_E_INVALIDSTREAMINDEX);
    RETURN_UNLESS(pdwFlags,E_POINTER);

   if (m_outputBuffer.getSize() || m_bDiscontinuityCalled)
    {
        *pdwFlags = 0; //The buffer still contains data; return zero.
    }
    else
    {
        *pdwFlags = DMO_INPUT_STATUSF_ACCEPT_DATA; // OK to call ProcessInput.
    }
    return S_OK;
}

/*  This function copies samples from m_inputBuffer into m_outputBuffer. 
    After each output sample it inserts an interpolated sample, thus doubling the sampling rate.
*/
void CSrDoublerWmpPlg::doubleSampleRate()
{
   CScrollBuffer scrollBuf(m_inputBuffer);
   if (scrollBuf.getNumFrames()<1)
      return ;
   const int nOutSampleSize=m_outputBuffer.getSampleSize();
   int nChannels=m_outputBuffer.getChannels();
   do {
      //expand the output buffer to add room for 2 new frames, 1 native and 1 interpolated
      m_outputBuffer.expand(2);
      //get the pointer on a new native frame cast as float
      float * pCurrentSample=(float*)&(m_outputBuffer[m_outputBuffer.getSize()-2]);
      
      //convert the native frame to float and copy it into the output buffer
      for (int channel=0;channel<nChannels;channel++)
      {
         double nativeSample=scrollBuf.getCurrentSample(channel);
         *pCurrentSample++=(float)nativeSample;
      }
      
      //now, get interpolated samples
      for (int channel=0;channel<nChannels;channel++)
      {         
         double interpSample= scrollBuf.getInterpolatedSample(channel);
         *pCurrentSample++=(float)interpSample;         
      }
   }
   while(scrollBuf.scroll());
}


/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::ProcessInput
//
// Implementation of IMediaObject::ProcessInput
/////////////////////////////////////////////////////////////////////////////
    
STDMETHODIMP CSrDoublerWmpPlg::ProcessInput( 
               DWORD dwInputStreamIndex,
               IMediaBuffer *pBuffer,
               DWORD dwFlags,
               REFERENCE_TIME rtTimestamp,
               REFERENCE_TIME rtTimelength)
{ 
    RETURN_UNLESS(m_bEnabled,S_FALSE);
    RETURN_UNLESS(dwInputStreamIndex==0, DMO_E_INVALIDSTREAMINDEX);
    RETURN_UNLESS(pBuffer,E_POINTER);
    RETURN_UNLESS(m_mtInput.majortype!=GUID_NULL,DMO_E_TYPE_NOT_SET);

    HRESULT hr = S_OK;
    // Get a pointer to the actual data and length information.
    BYTE    *pbInputData = NULL;
    DWORD   cbInputLength = 0;
    hr = pBuffer->GetBufferAndLength(&pbInputData, &cbInputLength);
    if (FAILED(hr))
        return hr;

    WAVEFORMATEX *pWave = ( WAVEFORMATEX * ) m_mtInput.pbFormat;
    int nInputSampleSize=pWave->wBitsPerSample/8;
    int nChannels=pWave->nChannels;
    int nInputFrameSize=nChannels*nInputSampleSize;
    bool bFloatInput= (pWave->wFormatTag==WAVE_FORMAT_IEEE_FLOAT) || 
        (pWave->wFormatTag==WAVE_FORMAT_EXTENSIBLE && ((WAVEFORMATEXTENSIBLE*)pWave)->SubFormat==KSDATAFORMAT_SUBTYPE_IEEE_FLOAT);
    int nOutputSampleSize=sizeof(float);
    
    //the samples in m_inputBuffer and in m_outputBuffer are retained
    //between ProcessInput calls. If any of them is empty, we 
    //initialize it.

    if (m_inputBuffer.getSize()==0)
    {//we are at the beginning of the playback
       m_inputBuffer.init(nInputSampleSize,nChannels,pWave->nSamplesPerSec,bFloatInput);
       m_inputBuffer.expand(TABLE_WIDTH/2-1);
    }
    
    if (m_outputBuffer.getSize()==0)
    {
        //the sample rate gets doubled, and format is always float
        m_outputBuffer.init(nOutputSampleSize,nChannels,pWave->nSamplesPerSec*2,true);
    }

    //copy the input data into the m_inputBuffer
    m_inputBuffer.append((const char*)pbInputData,cbInputLength/nInputFrameSize);
    
    if (m_inputBuffer.getSize()<TABLE_WIDTH)
       return S_FALSE;

    //Verify that buffer's time stamp is valid.
    if (dwFlags & DMO_INPUT_DATA_BUFFERF_TIME)
    {
        m_bValidTime = true;
        m_rtTimestamp = rtTimestamp;
    }
    else
    {
        m_bValidTime = false;
    }

    //call doubleSampleRate to add native and interpolated samples into the m_outputBuffer
    
    doubleSampleRate();
    
    //discard the data in the input buffer except for the last TABLE_WIDTH-1 frames
    //we will need those frame at the next ProcessInput call
    m_inputBuffer.truncate_beginning(m_inputBuffer.getSize()-(TABLE_WIDTH-1));
   
    return S_OK;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::ProcessOutput
//
// Implementation of IMediaObject::ProcessOutput
/////////////////////////////////////////////////////////////////////////////

STDMETHODIMP CSrDoublerWmpPlg::ProcessOutput( 
               DWORD dwFlags,
               DWORD cOutputBufferCount,
               DMO_OUTPUT_DATA_BUFFER *pOutputBuffers,
               DWORD *pdwStatus)
{
    RETURN_UNLESS(m_bEnabled,S_FALSE);
    HRESULT hr = S_OK;

    RETURN_UNLESS(pOutputBuffers,E_POINTER);
   
    // this plug-in only supports one output buffer
    RETURN_UNLESS(cOutputBufferCount==1,E_INVALIDARG);
    RETURN_UNLESS(m_mtOutput.majortype!=GUID_NULL,DMO_E_TYPE_NOT_SET);
   
    if (pdwStatus)
        *pdwStatus = 0;

    // make sure input and output buffers exist
    IMediaBuffer *pOutputBuffer = pOutputBuffers[0].pBuffer;

    if (!pOutputBuffer)
    {
        pOutputBuffers[0].dwStatus = 0;
        return S_FALSE;
    }
    
    if (m_bDiscontinuityCalled && m_inputBuffer.getSize())
    {//ensure smooth transition into silence
         WAVEFORMATEX *pWave = ( WAVEFORMATEX * ) m_mtInput.pbFormat;
         int curSize=m_inputBuffer.getSize();
         m_inputBuffer.expand(TABLE_WIDTH/2);

         doubleSampleRate();

         m_inputBuffer.clear();
    }
    BYTE       *pbOutputData = NULL;
    DWORD      cbOutputMaxLength = 0;
    int        cbBytesToProcess = 0;

    // Get current length of output buffer
    hr = pOutputBuffer->GetBufferAndLength(&pbOutputData, &cbOutputMaxLength);
    if (FAILED(hr))
        return hr;

    // Get max length of output buffer
    hr = pOutputBuffer->GetMaxLength(&cbOutputMaxLength);
    if (FAILED(hr))
        return hr;

    /* Calculate how many frames we can move from the m_outputBuffer into the pOutputBuffer
       and set the bComplete flag
    */
    bool bComplete = false; // The entire buffer is not yet processed.
    WAVEFORMATEX *pWave = ( WAVEFORMATEX * ) m_mtOutput.pbFormat;

    int nFramesToProcess=0;
    if (m_outputBuffer.getSize()*m_outputBuffer.getFrameSize()>(int)(cbOutputMaxLength))
    {//the m_outputBuffer does not fit into the pOutputBuffer
      nFramesToProcess=cbOutputMaxLength/m_outputBuffer.getFrameSize();
    }
    else
    {//the m_outputBuffer fits into the pOutputBuffer
      nFramesToProcess=m_outputBuffer.getSize();
      bComplete=true;
    }

    /* Move the frames from m_outputBuffer to pOutputBuffer */
    cbBytesToProcess=nFramesToProcess*m_outputBuffer.getFrameSize();
    if (cbBytesToProcess)
    {
       memcpy(pbOutputData, &m_outputBuffer[0], cbBytesToProcess);
#if DUMP_OUTPUT
       static SNDFILE * sf = NULL;
       if (!sf)
       {
          int sf_format = SF_FORMAT_WAV;
          if (m_outputBuffer.isFloat())
          {
             sf_format |= SF_FORMAT_FLOAT;
          }
          else
          {
             sf_format |= m_outputBuffer.getSampleSize();
          }
          SF_INFO info{ 0,m_outputBuffer.getSampleRate(),m_outputBuffer.getChannels(),sf_format,0,true };
          sf = sf_open("D:\\Debug\\dump.wav", SFM_WRITE, &info);
       }
       sf_write_raw(sf, pbOutputData, cbBytesToProcess);
#endif
    }
    hr = pOutputBuffer->SetLength(cbBytesToProcess);
    if (FAILED(hr))
        return hr;
    m_outputBuffer.truncate_beginning(nFramesToProcess);

    /* Set pOutputBuffer status and time fields */
    pOutputBuffers[0].dwStatus = 0;

    if (m_bValidTime)
    {
        // store start time of output buffer
        pOutputBuffers[0].dwStatus |= DMO_OUTPUT_DATA_BUFFERF_TIME;
        pOutputBuffers[0].rtTimestamp = m_rtTimestamp;
    
        // estimate time length of the output buffer
        pOutputBuffers[0].dwStatus |= DMO_OUTPUT_DATA_BUFFERF_TIMELENGTH;
        pOutputBuffers[0].rtTimelength = ::MulDiv(cbBytesToProcess, UNITS, pWave->nAvgBytesPerSec);

        // this much time has been consumed, so move the time stamp accordingly
        m_rtTimestamp += pOutputBuffers[0].rtTimelength;
    }

    if (bComplete) 
    {
        m_bValidTime = false;
        m_rtTimestamp = 0;
        m_bDiscontinuityCalled=false;
    }
    else 
    {
        // Let the client know there is still data that needs processing 
        // in the input buffer.
        pOutputBuffers[0].dwStatus |= DMO_OUTPUT_DATA_BUFFERF_INCOMPLETE;
    }
    return S_OK;
}

 
/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::Lock
//
// Implementation of IMediaObject::Lock
/////////////////////////////////////////////////////////////////////////////

STDMETHODIMP CSrDoublerWmpPlg::Lock( LONG bLock )
{
    return S_OK;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::SetEnable
//
// Implementation of IWMPPluginEnable::SetEnable
/////////////////////////////////////////////////////////////////////////////

STDMETHODIMP CSrDoublerWmpPlg::SetEnable( BOOL fEnable )
{
    // This function is called when the plug-in is being enabled or disabled,
    // typically by user action. Once a plug-in is disabled, it will still be
    // loaded into the graph but ProcessInput and ProcessOutput will not be called.

    // This function allows any state or UI associated with the plug-in to reflect the
    // enabled/disable state of the plug-in

   m_bEnabled = fEnable;

    return S_OK;
}

/////////////////////////////////////////////////////////////////////////////
// CSrDoublerWmpPlg::GetEnable
//
// Implementation of IWMPPluginEnable::GetEnable
/////////////////////////////////////////////////////////////////////////////

STDMETHODIMP CSrDoublerWmpPlg::GetEnable( BOOL *pfEnable )
{
    RETURN_UNLESS(pfEnable,E_POINTER);

    *pfEnable = m_bEnabled;

    return S_OK;
}


HRESULT CSrDoublerWmpPlg::ValidateInputMediaType(const DMO_MEDIA_TYPE *pmtTarget)
{
   RETURN_UNLESS(m_bEnabled,S_FALSE);
   if (!pmtTarget || !pmtTarget->pbFormat)
      return DMO_E_TYPE_NOT_ACCEPTED;

   WAVEFORMATEX *pWave = (WAVEFORMATEX *) pmtTarget->pbFormat;
   WAVEFORMATEXTENSIBLE *pWaveXT = (WAVEFORMATEXTENSIBLE *) pWave;
   
   if (pWave->wFormatTag!=WAVE_FORMAT_PCM && pWave->wFormatTag!=WAVE_FORMAT_EXTENSIBLE && pWave->wFormatTag!=WAVE_FORMAT_IEEE_FLOAT)
      return DMO_E_TYPE_NOT_ACCEPTED;

   bool bUseExtensible= pWave->wFormatTag==WAVE_FORMAT_EXTENSIBLE;

   bool bValidFormat=
        // make sure the target media type has the field values we require
         MEDIATYPE_Audio == pmtTarget->majortype  && 
         pmtTarget->formattype == FORMAT_WaveFormatEx  &&
         (pmtTarget->cbFormat >= ((bUseExtensible)?sizeof( WAVEFORMATEXTENSIBLE):sizeof( WAVEFORMATEX ))) &&
         pmtTarget->pbFormat &&
        
         // make sure the wave header has the field values we require
         pWave->nChannels && 
         pWave->nSamplesPerSec &&
         //we won't upsample high sample rate streams
         pWave->nSamplesPerSec<=50000 && 
         pWave->nAvgBytesPerSec && 
         pWave->nBlockAlign &&
         (pWave->wBitsPerSample == 16 || pWave->wBitsPerSample == 24 || pWave->wBitsPerSample == 32 );
    
   if (bUseExtensible)
   {   
      bValidFormat&= (pWaveXT->SubFormat == KSDATAFORMAT_SUBTYPE_PCM || pWaveXT->SubFormat == KSDATAFORMAT_SUBTYPE_IEEE_FLOAT);
   }

   return (bValidFormat)?S_OK:DMO_E_TYPE_NOT_ACCEPTED;
}

//Our output media type should always be WAVEFORMATEXTENSIBLE with a subformat KSDATAFORMAT_SUBTYPE_IEEE_FLOAT
HRESULT CSrDoublerWmpPlg::ValidateOutputMediaType(const DMO_MEDIA_TYPE *pmtTarget)
{
   RETURN_UNLESS(m_bEnabled,S_FALSE);
   if (!pmtTarget || !pmtTarget->pbFormat)
      return DMO_E_TYPE_NOT_ACCEPTED;

   WAVEFORMATEX *pWave = (WAVEFORMATEX *) pmtTarget->pbFormat;
   WAVEFORMATEXTENSIBLE *pWaveXT = (WAVEFORMATEXTENSIBLE *) pWave;
   bool bValidFormat=
        // make sure the target media type has the field values we require
         MEDIATYPE_Audio == pmtTarget->majortype  && 
         pmtTarget->formattype == FORMAT_WaveFormatEx  &&
         pmtTarget->cbFormat >= sizeof( WAVEFORMATEXTENSIBLE ) &&
         pmtTarget->pbFormat &&
        
         // make sure the wave header has the field values we require
         pWave->nChannels && 
         pWave->nSamplesPerSec &&
         pWave->nAvgBytesPerSec && 
         pWave->nBlockAlign &&
         pWave->wBitsPerSample == 32 && 
         pWave->wFormatTag == WAVE_FORMAT_EXTENSIBLE &&
         pWaveXT->SubFormat == KSDATAFORMAT_SUBTYPE_IEEE_FLOAT;

   return (bValidFormat)?S_OK:DMO_E_TYPE_NOT_ACCEPTED;
}

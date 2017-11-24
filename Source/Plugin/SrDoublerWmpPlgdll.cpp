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
// SrDoublerWmpPlgdll.cpp : Implementation of DLL Exports.

#include "stdafx.h"
#include "resource.h"
#include <initguid.h>
#include <Mfidl.h>
#include <mfapi.h>      // Media foundation API
#include <VersionHelpers.h>

#include "SrDoublerWmpPlg.h"

#include <uuids.h>      // DirectX SDK media types and subtyes
#include <dmoreg.h>     // DirectX SDK registration

DEFINE_GUID(CLSID_WMA_Decoder_DMO,0x2eeb4adf,0x4578,0x4d10,0xbc,0xa7,0xbb,0x95,0x5f,0x56,0x32,0x0a);
DEFINE_GUID(CLSID_MP3_Decoder_DMO,0xbbeea841,0x0a63,0x4f52,0xa7,0xab,0xa9,0xb3,0xa8,0x4e,0xd3,0x8a);
DEFINE_GUID(CLSID_WAV_Byte_Stream_Handler,0x42C9B9F5,0x16FC,0x47ef,0xAF,0x22,0xDA,0x05,0xF7,0xC8,0x42,0xE3);

//The Media Foundation pipeline does not support plug-ins that change audio stream format. 
//We need to force all files we care about to be played by the DirectShow pipeline
static class MFPluginDisabler
{
   typedef HRESULT (__stdcall * MFGetPluginControlType)( __out IMFPluginControl **ppPluginControl );
   MFGetPluginControlType m_pMFGetPluginControl;
   boolean m_bLibLoaded;
   HMODULE m_hModule;

   void DisableMFPlugins(BOOL bDisable)
   {
      IMFPluginControl * pControl = NULL;
      if (m_pMFGetPluginControl)
         (*m_pMFGetPluginControl)(&pControl);
      if (pControl)
      {
            //Disable MF codecs for WMA, MP3 and WAV files
         pControl->SetDisabled(MF_Plugin_Type_MFT,CLSID_WMA_Decoder_DMO,bDisable);
         pControl->SetDisabled(MF_Plugin_Type_MFT,CLSID_MP3_Decoder_DMO,bDisable);
         pControl->SetDisabled(MF_Plugin_Type_MediaSource,CLSID_WAV_Byte_Stream_Handler,bDisable);
         pControl->Release();
         pControl = NULL;
      }
   }

public:
   MFPluginDisabler() 
   {
      m_pMFGetPluginControl = NULL;
      const char * LIB_NAME = "mfplat.dll";
      m_bLibLoaded = false;
      m_hModule = GetModuleHandle(LIB_NAME);
      if (!m_hModule) 
      {//load the library
         m_hModule = LoadLibrary(LIB_NAME);
         m_bLibLoaded = (m_hModule!=NULL);
      }
      if (m_hModule) 
         m_pMFGetPluginControl = (MFGetPluginControlType)GetProcAddress(m_hModule,"MFGetPluginControl");

      DisableMFPlugins(TRUE);
   }
   ~MFPluginDisabler() 
   {
      DisableMFPlugins(FALSE);
      if (m_bLibLoaded)
         FreeLibrary(m_hModule);
   }
} disabler;



CComModule _Module;

BEGIN_OBJECT_MAP(ObjectMap)
OBJECT_ENTRY(CLSID_SrDoublerWmpPlg, CSrDoublerWmpPlg)
END_OBJECT_MAP()

/////////////////////////////////////////////////////////////////////////////
// DLL Entry Point
/////////////////////////////////////////////////////////////////////////////

extern "C"
BOOL WINAPI DllMain(HINSTANCE hInstance, DWORD dwReason, LPVOID /*lpReserved*/)
{
    if (dwReason == DLL_PROCESS_ATTACH)
    {
        _Module.Init(ObjectMap, hInstance);
        DisableThreadLibraryCalls(hInstance);
    }
    else if (dwReason == DLL_PROCESS_DETACH)
    {
        _Module.Term();
    }
   
    return TRUE;    // ok
}

/////////////////////////////////////////////////////////////////////////////
// Used to determine whether the DLL can be unloaded by OLE
/////////////////////////////////////////////////////////////////////////////

STDAPI DllCanUnloadNow(void)
{
    return (_Module.GetLockCount()==0) ? S_OK : S_FALSE;
}

/////////////////////////////////////////////////////////////////////////////
// Returns a class factory to create an object of the requested type
/////////////////////////////////////////////////////////////////////////////

STDAPI DllGetClassObject(REFCLSID rclsid, REFIID riid, LPVOID* ppv)
{
    return _Module.GetClassObject(rclsid, riid, ppv);
}

/////////////////////////////////////////////////////////////////////////////
// DllRegisterServer - Adds entries to the system registry
/////////////////////////////////////////////////////////////////////////////

STDAPI DllRegisterServer(void)
{
    CComPtr<IWMPMediaPluginRegistrar> spRegistrar;
    HRESULT hr;

    // Create the registration object
    hr = spRegistrar.CoCreateInstance(CLSID_WMPMediaPluginRegistrar, NULL, CLSCTX_INPROC_SERVER);
    if (FAILED(hr))
    {
       return hr;
    }
    
    // Load friendly name and description strings
    CComBSTR    bstrFriendlyName;
    CComBSTR    bstrDescription;

    bstrFriendlyName.LoadString(IDS_FRIENDLYNAME);
    bstrDescription.LoadString(IDS_DESCRIPTION);

    // Describe the type of data handled by the plug-in
    DMO_PARTIAL_MEDIATYPE mt[2] = { 0 ,0};
    mt[0].type = MEDIATYPE_Audio;
    mt[0].subtype = MEDIASUBTYPE_PCM;
    mt[1].type = MEDIATYPE_Audio;
    mt[1].subtype = MEDIASUBTYPE_IEEE_FLOAT;

    // Register the plug-in with WMP
    hr = spRegistrar->WMPRegisterPlayerPlugin(
                    bstrFriendlyName,   // friendly name (for menus, etc)
                    bstrDescription,    // description (for Tools->Options->Plug-ins)
                    NULL,               // path to app that uninstalls the plug-in
                    10,                  // DirectShow priority for this plug-in
                    WMP_PLUGINTYPE_DSP, // Plug-in type
                    CLSID_SrDoublerWmpPlg,// Class ID of plug-in
                    2,                  // No. media types supported by plug-in
                    &mt);               // Array of media types supported by plug-in

    // Also register for out-of-proc playback in the MF pipeline
    // We'll only do this on Windows Vista or later operating systems because
    // WMP 11 and Vista are required at a minimum.
    if (SUCCEEDED(hr) && 
        TRUE == IsWindowsVistaOrGreater())
    {
        hr = spRegistrar->WMPRegisterPlayerPlugin(
                        bstrFriendlyName,   // friendly name (for menus, etc)
                        bstrDescription,    // description (for Tools->Options->Plug-ins)
                        NULL,               // path to app that uninstalls the plug-in
                        10,                  // DirectShow priority for this plug-in
                        WMP_PLUGINTYPE_DSP_OUTOFPROC, // Plug-in type
                        CLSID_SrDoublerWmpPlg,// Class ID of plug-in
                        2,                  // No. media types supported by plug-in
                        &mt);               // Array of media types supported by plug-in
    }

   if (FAILED(hr))
    {
       return hr;
    }

    // registers object, typelib and all interfaces in typelib
    return _Module.RegisterServer();
}

/////////////////////////////////////////////////////////////////////////////
// DllUnregisterServer - Removes entries from the system registry
/////////////////////////////////////////////////////////////////////////////

STDAPI DllUnregisterServer(void)

{
    CComPtr<IWMPMediaPluginRegistrar> spRegistrar;
    HRESULT hr;

    // Create the registration object
    hr = spRegistrar.CoCreateInstance(CLSID_WMPMediaPluginRegistrar, NULL, CLSCTX_INPROC_SERVER);
    if (FAILED(hr))
    {
        return hr;
    }

    // Tell WMP to remove this plug-in
    hr = spRegistrar->WMPUnRegisterPlayerPlugin(WMP_PLUGINTYPE_DSP, CLSID_SrDoublerWmpPlg);

    if(TRUE == IsWindowsVistaOrGreater())
    {
        // Also unregister from the MF pipeline
        hr = spRegistrar->WMPUnRegisterPlayerPlugin(WMP_PLUGINTYPE_DSP_OUTOFPROC, CLSID_SrDoublerWmpPlg);
    }

    return _Module.UnregisterServer();
}


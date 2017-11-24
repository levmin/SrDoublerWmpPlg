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

#pragma once

#define WIN32_LEAN_AND_MEAN      // Exclude rarely-used stuff from Windows headers
#define _ATL_CSTRING_EXPLICIT_CONSTRUCTORS   // some CString constructors will be explicit

#ifndef VC_EXTRALEAN
#define VC_EXTRALEAN      // Exclude rarely-used stuff from Windows headers
#endif

#include <string>
#include <stdio.h>
#include <windows.h>
#include <shellapi.h>
#include <shlobj.h>
#include <psapi.h>





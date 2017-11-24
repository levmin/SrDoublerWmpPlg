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
/*   SrDoublerWmpPlg Uninstall
*/
#include "stdafx.h"
#include <vector>
#include <fstream>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

using namespace std;

void AddTrailingSeparator(string& str)
{
   if (str.length()==0 || *(str.end()-1)!='\\')
      str+="\\";
}

int UnregisterDLLs(string installFolder, const vector<string>& names)
{
   AddTrailingSeparator(installFolder);

   for (const string& name: names)
   {
      string commandLine = " /u /s \"" + installFolder + name + "\"";
      ShellExecute(NULL,NULL,"regsvr32.exe",commandLine.c_str(),NULL,SW_HIDE);
   }
   return 0;
}

void GetEnvVar(const char * varName, string& var)
{
    const int BUF_LEN=32767; 
    char buf[BUF_LEN]={0}; 
    GetEnvironmentVariable(varName,buf,BUF_LEN);
    var = buf;
}

int WINAPI WinMain(HINSTANCE,HINSTANCE,LPSTR,int)
{
   string programFiles;
   GetEnvVar("ProgramFiles",programFiles);
   AddTrailingSeparator(programFiles);
   string installFolder=programFiles+"SrDoublerWmpPlg";

   const vector<string> names={
      "SrDoublerWmpPlg.dll",
   };
   if (UnregisterDLLs(installFolder,names)!=0)
      return -1;
    
   const char * uninstKey="SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\SrDoublerWmpPlg";
   RegDeleteKey(HKEY_LOCAL_MACHINE,uninstKey);
   char szTmpPath[MAX_PATH];
   GetTempPath(MAX_PATH,szTmpPath);
   string tmpDir(szTmpPath);
   AddTrailingSeparator(tmpDir);
   string tmpBatFile=tmpDir+"finish_cleanup.bat";
   string uninstallExe=installFolder+"\\Uninstall.exe";
   ofstream f;
   f.open(tmpBatFile.c_str(),ios::out | ios::trunc);
   f   << "echo off\n"
      << ":wait\n"
      << "del \"" + uninstallExe + "\"\n"
      << "if exist \"" + uninstallExe + "\" goto wait\n"
      << "rd /s /q \"" + installFolder + "\"\n"
      << "del \"" + tmpDir + "finish_cleanup.bat\"\n";
   f.close();

   ShellExecute(NULL, NULL, tmpBatFile.c_str(), NULL, NULL, SW_HIDE);

   return 0;
}

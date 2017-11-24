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

/*   SrDoublerWmpPlg Install
*/

#include "stdafx.h"
#include <vector>

#ifdef _DEBUG
#define new DEBUG_NEW
#endif

using namespace std;

void AddTrailingSeparator(string& str)
{
   if (str.length()==0 || *(str.end()-1)!='\\')
      str+="\\";
}

int CopyFiles(string installFolder, const vector<string>& names )
{
   AddTrailingSeparator(installFolder);

   for(const string & name : names )
   {
      DWORD attr=0;
      const string&& fileName = installFolder+name;
      if (!CopyFile(name.c_str(),fileName.c_str(),FALSE)
          || (attr=GetFileAttributes(fileName.c_str())==INVALID_FILE_ATTRIBUTES)
          || !SetFileAttributes(fileName.c_str(),attr & ~FILE_ATTRIBUTE_READONLY))
         {
            MessageBox(NULL,("Copy operation failed for "+name).c_str(),NULL,MB_OK);
            return -1;
         }
      
   }
   return 0;
}

int RegisterDLLs(string installFolder, const vector<string>& names)
{
   AddTrailingSeparator(installFolder);
   for (const string & name : names)
   {
      string commandLine = " /s \"" + installFolder + name +"\"";
      ShellExecute(NULL,NULL,"regsvr32.exe",commandLine.c_str(),NULL,SW_HIDE);
   }
   return 0;
}

void SetFileDirectory()
{
   char image_dir[MAX_PATH] = {0};
   GetModuleFileNameEx(GetCurrentProcess(),GetModuleHandle(NULL),image_dir,MAX_PATH);
   string imageDir(image_dir);
   int nPos = imageDir.find_last_of('\\');
   if (nPos < 0)
      return; //should not happen
   imageDir = imageDir.substr(0,nPos+1);
   SetCurrentDirectory(imageDir.c_str());
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
   //set current directory to be the one of install.exe
   //this is needed because CopyFiles assumes the files are in the current directory
   SetFileDirectory();
   string programFiles;
   GetEnvVar("ProgramFiles",programFiles);
   AddTrailingSeparator(programFiles);
   string installFolder=programFiles+"SrDoublerWmpPlg";
      
   CreateDirectory(installFolder.c_str(),NULL);
   
   const vector<string> fileNames={
        "SrDoublerWmpPlg.dll",
        "UnInstall.exe",
    };
    
    const vector<string> dllNames={
        "SrDoublerWmpPlg.dll",
    };
    
    if (CopyFiles(installFolder,fileNames)!=0 ||
        RegisterDLLs(installFolder,dllNames)!=0)
        return -1;

   HKEY key = NULL;
   REGSAM access = KEY_ALL_ACCESS;
   RegCreateKeyEx(HKEY_LOCAL_MACHINE,"SOFTWARE\\Microsoft\\Windows\\CurrentVersion\\Uninstall\\SrDoublerWmpPlg",0,0,0,access,0,&key,NULL);
   const BYTE displayName[]="SrDoublerWmpPlg";
   RegSetValueEx(key,"DisplayName",0,REG_SZ,&displayName[0],sizeof(displayName)/sizeof(BYTE));
   string uninstallString=installFolder;
   AddTrailingSeparator(uninstallString);
   uninstallString+="Uninstall.exe";
   RegSetValueEx(key,"UninstallString",0,REG_SZ,(const BYTE *)uninstallString.c_str(),uninstallString.length());
   MessageBox(NULL,"SrDoublerWmpPlg is successfully installed","SrDoublerWmpPlg Installer",MB_OK);
   return 0;

}

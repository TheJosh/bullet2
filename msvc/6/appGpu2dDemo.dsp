# Microsoft Developer Studio Project File - Name="appGpu2dDemo" - Package Owner=<4>
# Microsoft Developer Studio Generated Build File, Format Version 6.00
# ** DO NOT EDIT **

# TARGTYPE "Win32 (x86) Console Application" 0x0103

CFG=appGpu2dDemo - Win32 Release
!MESSAGE This is not a valid makefile. To build this project using NMAKE,
!MESSAGE use the Export Makefile command and run
!MESSAGE 
!MESSAGE NMAKE /f "appGpu2dDemo.mak".
!MESSAGE 
!MESSAGE You can specify a configuration when running NMAKE
!MESSAGE by defining the macro CFG on the command line. For example:
!MESSAGE 
!MESSAGE NMAKE /f "appGpu2dDemo.mak" CFG="appGpu2dDemo - Win32 Release"
!MESSAGE 
!MESSAGE Possible choices for configuration are:
!MESSAGE 
!MESSAGE "appGpu2dDemo - Win32 DebugDoublePrecision" (based on "Win32 (x86) Console Application")
!MESSAGE "appGpu2dDemo - Win32 DebugDll" (based on "Win32 (x86) Console Application")
!MESSAGE "appGpu2dDemo - Win32 Debug" (based on "Win32 (x86) Console Application")
!MESSAGE "appGpu2dDemo - Win32 ReleaseDoublePrecision" (based on "Win32 (x86) Console Application")
!MESSAGE "appGpu2dDemo - Win32 ReleaseDll" (based on "Win32 (x86) Console Application")
!MESSAGE "appGpu2dDemo - Win32 Release" (based on "Win32 (x86) Console Application")
!MESSAGE 

# Begin Project
# PROP AllowPerConfigDependencies 0
# PROP Scc_ProjName ""
# PROP Scc_LocalPath ""
CPP=cl.exe
MTL=midl.exe
RSC=rc.exe

!IF  "$(CFG)" == "appGpu2dDemo - Win32 Release"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 0
# PROP BASE Output_Dir "release"
# PROP BASE Intermediate_Dir "release"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 0
# PROP Output_Dir "..\..\out\release6\build\appGpu2dDemo\"
# PROP Intermediate_Dir "..\..\out\release6\build\appGpu2dDemo\"
# PROP Ignore_Export_Lib 0
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /Gy /GF /MD /Ob2 /Zm1000 /Og /Oi /Ot /Oy /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut" /I "..\..\Demos\OpenGL" /I "..\..\Extras\ConvexHull"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release6\build\appGpu2dDemo\appGpu2dDemo.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut" /i "..\..\Demos\OpenGL" /i "..\..\Extras\ConvexHull"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386 /OPT:NOREF /out:"..\..\Gpu2dDemo.exe" /subsystem:console /libpath:"..\..\Glut" 

!ELSEIF  "$(CFG)" == "appGpu2dDemo - Win32 ReleaseDll"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "release_dll"
# PROP BASE Intermediate_Dir "release_dll"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\release_dll6\build\appGpu2dDemo\"
# PROP Intermediate_Dir "..\..\out\release_dll6\build\appGpu2dDemo\"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c  /D "_MT" /D "_MBCS" /D "NDEBUG" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut" /I "..\..\Demos\OpenGL" /I "..\..\Extras\ConvexHull"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release_dll6\build\appGpu2dDemo\appGpu2dDemo.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut" /i "..\..\Demos\OpenGL" /i "..\..\Extras\ConvexHull"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386  /out:"..\..\Gpu2dDemo.exe" /subsystem:console /libpath:"..\..\Glut" 

!ELSEIF  "$(CFG)" == "appGpu2dDemo - Win32 ReleaseDoublePrecision"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "release_dbl"
# PROP BASE Intermediate_Dir "release_dbl"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\release_dbl6\build\appGpu2dDemo\"
# PROP Intermediate_Dir "..\..\out\release_dbl6\build\appGpu2dDemo\"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c  /D "_MT" /D "_MBCS" /D "NDEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut" /I "..\..\Demos\OpenGL" /I "..\..\Extras\ConvexHull"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "NDEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\release_dbl6\build\appGpu2dDemo\appGpu2dDemo.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut" /i "..\..\Demos\OpenGL" /i "..\..\Extras\ConvexHull"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386  /out:"..\..\Gpu2dDemo.exe" /subsystem:console /libpath:"..\..\Glut" 

!ELSEIF  "$(CFG)" == "appGpu2dDemo - Win32 Debug"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 1
# PROP BASE Output_Dir "debug"
# PROP BASE Intermediate_Dir "debug"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 1
# PROP Output_Dir "..\..\out\debug6\build\appGpu2dDemo\"
# PROP Intermediate_Dir "..\..\out\debug6\build\appGpu2dDemo\"
# PROP Ignore_Export_Lib 1
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c /GR /MDd /Zm1000 /ZI /Od /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut" /I "..\..\Demos\OpenGL" /I "..\..\Extras\ConvexHull"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug6\build\appGpu2dDemo\appGpu2dDemo.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut" /i "..\..\Demos\OpenGL" /i "..\..\Extras\ConvexHull"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386 /debug /pdbtype:sept /out:"..\..\Gpu2dDemo.exe" /subsystem:console /libpath:"..\..\Glut" 

!ELSEIF  "$(CFG)" == "appGpu2dDemo - Win32 DebugDll"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "debug_dll"
# PROP BASE Intermediate_Dir "debug_dll"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\debug_dll6\build\appGpu2dDemo\"
# PROP Intermediate_Dir "..\..\out\debug_dll6\build\appGpu2dDemo\"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c  /D "_MT" /D "_MBCS" /D "_DEBUG" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut" /I "..\..\Demos\OpenGL" /I "..\..\Extras\ConvexHull"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug_dll6\build\appGpu2dDemo\appGpu2dDemo.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut" /i "..\..\Demos\OpenGL" /i "..\..\Extras\ConvexHull"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386  /out:"..\..\Gpu2dDemo.exe" /subsystem:console /libpath:"..\..\Glut" 

!ELSEIF  "$(CFG)" == "appGpu2dDemo - Win32 DebugDoublePrecision"

# PROP BASE Use_MFC 0
# PROP BASE Use_Debug_Libraries 
# PROP BASE Output_Dir "debug_dbl"
# PROP BASE Intermediate_Dir "debug_dbl"
# PROP BASE Target_Dir ""
# PROP Use_MFC 0
# PROP Use_Debug_Libraries 
# PROP Output_Dir "..\..\out\debug_dbl6\build\appGpu2dDemo\"
# PROP Intermediate_Dir "..\..\out\debug_dbl6\build\appGpu2dDemo\"
# PROP Target_Dir ""
# ADD BASE CPP /nologo /vmb /vms /W3 /Gm /G5 /D "WIN32" /FD /c
# ADD CPP /nologo /vmb /vms /W3 /Gm /G5 /FD /c  /D "_MT" /D "_MBCS" /D "_DEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE" /D "WIN32"  /I "." /I "..\.." /I "..\..\src" /I "..\..\Glut" /I "..\..\Demos\OpenGL" /I "..\..\Extras\ConvexHull"
# ADD BASE MTL /nologo /mktyplib203 /o "NUL" /win32
# ADD MTL /nologo /mktyplib203 /o "NUL" /win32 /D "_DEBUG" /D "BT_USE_DOUBLE_PRECISION" /D "_CONSOLE" /D "_CRT_SECURE_NO_DEPRECATE" /D "_CRT_NONSTDC_NO_DEPRECATE"
# ADD BASE RSC /l 0x409
# ADD RSC /l 0x409 /fo".\..\..\out\debug_dbl6\build\appGpu2dDemo\appGpu2dDemo.res" /i "." /i "..\.." /i "..\..\src" /i "..\..\Glut" /i "..\..\Demos\OpenGL" /i "..\..\Extras\ConvexHull"
BSC32=bscmake.exe
# ADD BASE BSC32 /nologo
# ADD BSC32 /nologo
LIB32=link.exe -lib
# ADD BASE LIB32 /nologo
# ADD LIB32 /nologo 
LINK32=link.exe
# ADD BASE LINK32 user32.lib gdi32.lib advapi32.lib /nologo /machine:I386
# ADD LINK32 shell32.lib user32.lib gdi32.lib advapi32.lib glut32.lib  /nologo /version:4.0 /machine:I386  /out:"..\..\Gpu2dDemo.exe" /subsystem:console /libpath:"..\..\Glut" 

!ENDIF 

# Begin Target

# Name "appGpu2dDemo - Win32 Release"
# Name "appGpu2dDemo - Win32 ReleaseDll"
# Name "appGpu2dDemo - Win32 ReleaseDoublePrecision"
# Name "appGpu2dDemo - Win32 Debug"
# Name "appGpu2dDemo - Win32 DebugDll"
# Name "appGpu2dDemo - Win32 DebugDoublePrecision"
# Begin Group "Source Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Demos\Gpu2dDemo\BasicDemo.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Demos\Gpu2dDemo\btGpuDemo2dCpuFunc.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Demos\Gpu2dDemo\btGpuDemoDynamicsWorld.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Demos\Gpu2dDemo\btGpuDemoPairCache.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Demos\Gpu2dDemo\main.cpp
# End Source File
# Begin Source File

SOURCE=..\..\Demos\Gpu2dDemo\oecakeLoader.cpp
# End Source File
# End Group
# Begin Group "Header Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\Demos\Gpu2dDemo\BasicDemo.h
# End Source File
# Begin Source File

SOURCE=..\..\Demos\Gpu2dDemo\btGpuDefines.h
# End Source File
# Begin Source File

SOURCE=..\..\Demos\Gpu2dDemo\btGpuDemo2dSharedCode.h
# End Source File
# Begin Source File

SOURCE=..\..\Demos\Gpu2dDemo\btGpuDemo2dSharedDefs.h
# End Source File
# Begin Source File

SOURCE=..\..\Demos\Gpu2dDemo\btGpuDemo2dSharedTypes.h
# End Source File
# Begin Source File

SOURCE=..\..\Demos\Gpu2dDemo\btGpuDemoDynamicsWorld.h
# End Source File
# Begin Source File

SOURCE=..\..\Demos\Gpu2dDemo\btGpuDemoPairCache.h
# End Source File
# Begin Source File

SOURCE=..\..\Demos\Gpu2dDemo\oecakeLoader.h
# End Source File
# End Group
# Begin Group "Resource Files"

# PROP Default_Filter ""
# Begin Source File

SOURCE=..\..\msvc\appGpu2dDemo.rc
# End Source File
# End Group
# End Target
# End Project

echo on

rem  temporarily add 64 bit compiler directory to path
set PATH=C:\mingw64\bin;%PATH%

minGW32-make clean

minGW32-make all
pause
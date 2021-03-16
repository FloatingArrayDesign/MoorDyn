echo off

rem  temporarily add 32 bit compiler directory to path
set PATH=C:\minGW32\bin;%PATH%

minGW32-make clean

minGW32-make
pause
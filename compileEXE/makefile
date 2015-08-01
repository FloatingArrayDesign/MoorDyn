# makefile for MoorDyn compiled with driver program as an executable


# tell to search for dependencies in parent dir
VPATH = ../source
	
CFLAGS2 = -g -O3 -w -Wall -static -static-libgcc -static-libstdc++ -std=gnu++0x 

CFLAGS = -c -O3 -g -w -Wall -static -static-libgcc -static-libstdc++ -std=gnu++0x 


all: MoorDyn_Driver_PtfmMotion.exe
 
MoorDyn_Driver_PtfmMotion.exe: MoorDyn_driver.o MoorDyn.o Line.o Connection.o Misc.o kiss_fft.o
	g++ $(CFLAGS2) -o MoorDyn_Driver_PtfmMotion.exe MoorDyn_driver.o MoorDyn.o Line.o Connection.o Misc.o kiss_fft.o -lopengl32

MoorDyn_driver.o: MoorDyn_driver.cpp MoorDyn.h
	g++ $(CFLAGS) ../source/MoorDyn_driver.cpp
	
MoorDyn.o: MoorDyn.cpp MoorDyn.h Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h Misc.cpp kiss_fft.h
	g++ $(CFLAGS) ../source/MoorDyn.cpp
	
kiss_fft.o: kiss_fft.h kiss_fft.c
	g++ $(CFLAGS) ../source/kiss_fft.c
	
Line.o: Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h
	g++ $(CFLAGS) ../source/Line.cpp

Connection.o: Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h Misc.cpp
	g++ $(CFLAGS) ../source/Connection.cpp
	
Misc.o: Misc.h Misc.cpp
	g++ $(CFLAGS) ../source/Misc.cpp

clean:
	del *.exe.manifest *.exe *.obj *.o *.dll

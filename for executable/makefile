# makefile for MoorDyn compiled with driver program as an executable
	
CFLAGS2 = -static -static-libgcc -static-libstdc++

CFLAGS = -c -g -O3 -w -Wall -static -static-libgcc -static-libstdc++ -std=gnu++0x 

# excess of "static" flags above is to avoid DLL dependencies (it probably needs cleanup)

all: MoorDyn_Driver_PtfmMotion.exe
 
MoorDyn_Driver_PtfmMotion.exe: MoorDyn_driver.o main.o Line.o Connection.o Misc.o
	g++ $(CFLAGS2) -o MoorDyn_Driver_PtfmMotion.exe MoorDyn_driver.o main.o Line.o Connection.o Misc.o

MoorDyn_driver.o: MoorDyn_driver.cpp main.h
	g++ $(CFLAGS) MoorDyn_driver.cpp
	
main.o: main.cpp main.h Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h Misc.cpp
	g++ $(CFLAGS) main.cpp
	
Line.o: Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h
	g++ $(CFLAGS) Line.cpp

Connection.o: Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h Misc.cpp
	g++ $(CFLAGS) Connection.cpp
	
Misc.o: Misc.h Misc.cpp
	g++ $(CFLAGS) Misc.cpp

clean:
	del *.exe.manifest *.exe *.obj *.o *.dll

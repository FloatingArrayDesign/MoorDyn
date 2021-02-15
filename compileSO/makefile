# makefile for MoorDyn compiled as MoorDyn.so (Linux)


# tell to search for dependencies in parent dir
VPATH = ../

LFLAGS = -shared -static-libgcc -static-libstdc++ -DMoorDyn_EXPORTS -DLINUX -fPIC

CFLAGS = -c -O3 -g -w -Wall -static -static-libgcc -static-libstdc++ -std=gnu++0x -DLINUX -fPIC


all: MoorDyn.so
 
MoorDyn.so: MoorDyn.o Line.o Connection.o Misc.o kiss_fft.o
	g++ $(LFLAGS) -o MoorDyn.so MoorDyn.o Line.o Connection.o Misc.o kiss_fft.o

MoorDyn.o: MoorDyn.cpp MoorDyn.h Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h Misc.cpp
	g++ $(CFLAGS) ../MoorDyn.cpp
	
kiss_fft.o: kiss_fft.h kiss_fft.c
	g++ $(CFLAGS) ../kiss_fft.c
	
Line.o: Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h
	g++ $(CFLAGS) ../Line.cpp

Connection.o: Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h Misc.cpp
	g++ $(CFLAGS) ../Connection.cpp
	
Misc.o: Misc.h Misc.cpp
	g++ $(CFLAGS) ../Misc.cpp

clean:
	rm *.o *.so


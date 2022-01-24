# makefile for MoorDyn compiled as Lines.dll


# tell to search for dependencies in parent dir
VPATH = ../source

LFLAGS = -shared -static-libgcc -static-libstdc++ -DMoorDyn_EXPORTS -DLINUX -fPIC

CFLAGS = -c -O3 -g -w -Wall -static -static-libgcc -static-libstdc++ -std=gnu++0x -DLINUX -fPIC


all: Lines.so
 
Lines.so: MoorDyn.o Line.o Connection.o Misc.o kiss_fft.o
	g++ $(LFLAGS) -o Lines.so MoorDyn.o Line.o Connection.o Misc.o kiss_fft.o

MoorDyn.o: ../../source/MoorDyn.cpp ../../source/MoorDyn.h ../../source/Line.h ../../source/Connection.h ../../source/QSlines.h ../../source/Misc.h
	g++ $(CFLAGS) ../../source/MoorDyn.cpp
	
kiss_fft.o: ../../source/kiss_fft.c ../../source/kiss_fft.h
	g++ $(CFLAGS) ../../source/kiss_fft.c
	
Line.o: ../../source/Line.cpp ../../source/Line.h ../../source/Connection.h ../../source/QSlines.h ../../source/Misc.h
	g++ $(CFLAGS) ../../source/Line.cpp

Connection.o: ../../source/Connection.cpp ../../source/Line.h ../../source/Connection.h ../../source/QSlines.h ../../source/Misc.h
	g++ $(CFLAGS) ../../source/Connection.cpp
	
Misc.o: ../../source/Misc.cpp ../../source/Misc.h
	g++ $(CFLAGS) ../../source/Misc.cpp

clean:
	rm -f *.o *.so


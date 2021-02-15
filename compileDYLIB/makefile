# makefile for MoorDyn compiled as MoorDyn.dylib


# tell to search for dependencies in parent dir
VPATH = ../

#LFLAGS = -shared -static -static-libgcc -static-libstdc++ -DMoorDyn_EXPORTS
LFLAGS = -shared -DMoorDyn_EXPORTS

CFLAGS = -c -Ofast -g -w -Wall -static -static-libgcc -static-libstdc++ -std=gnu++0x -DMoorDyn_EXPORTS -DOSX

CC = clang++

all: MoorDyn.dylib
 
MoorDyn.dylib: MoorDyn.o Line.o Connection.o Misc.o kiss_fft.o
	$(CC) $(LFLAGS) -o $@ MoorDyn.o Line.o Connection.o Misc.o kiss_fft.o
# added linking to openGL above

MoorDyn.o: MoorDyn.cpp MoorDyn.h Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h Misc.cpp
	$(CC) $(CFLAGS) ../MoorDyn.cpp
	
kiss_fft.o: kiss_fft.h kiss_fft.c
	$(CC) $(CFLAGS) ../kiss_fft.c
	
Line.o: Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h
	$(CC) $(CFLAGS) ../Line.cpp

Connection.o: Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h Misc.cpp
	$(CC) $(CFLAGS) ../Connection.cpp
	
Misc.o: Misc.h Misc.cpp
	$(CC) $(CFLAGS) ../Misc.cpp

clean:
	rm  *.o *.dylib


# makefile for MoorDyn compiled as Lines.dll


# tell to search for dependencies in parent dir
VPATH = ../source

LFLAGS = -shared -static-libgcc -static-libstdc++ -DMoorDyn_EXPORTS -DLINUX -fPIC

CFLAGS = -c -O3 -g -w -Wall -static -static-libgcc -static-libstdc++ -std=gnu++0x -DLINUX -fPIC


all: libmoordyn.so

libmoordyn.so: Lines.so
	rm -f libmoordyn.so
	ln -s Lines.so libmoordyn.so

Lines.so: MoorDyn.o Line.o Connection.o Body.o Rod.o Waves.o Misc.o kiss_fft.o kiss_fftr.o
	g++ $(LFLAGS) -o Lines.so MoorDyn.o Line.o Connection.o Body.o Rod.o Waves.o Misc.o kiss_fft.o kiss_fftr.o

MoorDyn.o: ../../source/MoorDyn.cpp ../../source/MoorDyn.h ../../source/Line.h ../../source/Connection.h ../../source/QSlines.h ../../source/Misc.h
	g++ $(CFLAGS) ../../source/MoorDyn.cpp

kiss_fft.o: ../../source/kiss_fft.c ../../source/kiss_fft.h
	g++ $(CFLAGS) ../../source/kiss_fft.c

kiss_fftr.o: ../../source/kiss_fftr.c ../../source/kiss_fftr.h
	g++ $(CFLAGS) ../../source/kiss_fftr.c

Line.o: ../../source/Line.cpp ../../source/Line.h ../../source/Connection.h ../../source/QSlines.h ../../source/Misc.h
	g++ $(CFLAGS) ../../source/Line.cpp

Connection.o: ../../source/Connection.cpp ../../source/Line.h ../../source/Connection.h ../../source/QSlines.h ../../source/Misc.h
	g++ $(CFLAGS) ../../source/Connection.cpp

Body.o: ../../source/Body.cpp ../../source/Body.h
	g++ $(CFLAGS) ../../source/Body.cpp

Rod.o: ../../source/Rod.cpp ../../source/Rod.h
	g++ $(CFLAGS) ../../source/Rod.cpp

Waves.o: ../../source/Waves.cpp ../../source/Waves.h
	g++ $(CFLAGS) ../../source/Waves.cpp

Misc.o: ../../source/Misc.cpp ../../source/Misc.h
	g++ $(CFLAGS) ../../source/Misc.cpp

clean:
	rm -f *.o *.so
	rm -f minimal 

test: ../../test/minimal.cpp libmoordyn.so ../../source/MoorDyn.h ../../source/Misc.h
	g++ -L./ -lmoordyn -I../../source/ -DLINUX -o minimal ../../test/minimal.cpp
	./minimal

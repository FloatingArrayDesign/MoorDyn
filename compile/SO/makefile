# makefile for MoorDyn compiled as Lines.so, and creating a convenient symbolic
# link as libmoordyn.so (so it can be linked with -lmoordyn)
#
# The following targets are provided:
#
# all: Builds the library Lines.so, and the symbolic link
#      libmoordyn.so -> Lines.so
#
# test: Build libmoordyn.so, if not yet built, and then the tests:
#        - test_minimal
#
# debug: Build both targets, all and test, with debug information


# tell to search for dependencies in parent dir
VPATH = ../source

LFLAGS = -shared -static-libgcc -static-libstdc++ -DMoorDyn_EXPORTS -DLINUX -fPIC

CFLAGS = -c -O3 -w -Wall -std=gnu++0x -DLINUX -fPIC
EXEFLAGS = -DLINUX

all: libmoordyn.so

libmoordyn.so: Lines.so
	rm -f libmoordyn.so
	ln -s Lines.so libmoordyn.so

Lines.so: MoorDyn.o Line.o Connection.o Body.o Rod.o Waves.o Misc.o kiss_fft.o kiss_fftr.o
	gcc $(LFLAGS) -o Lines.so MoorDyn.o Line.o Connection.o Body.o Rod.o Waves.o Misc.o kiss_fft.o kiss_fftr.o

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
	rm -f test_minimal 

test_minimal: ../../test/minimal.cpp libmoordyn.so ../../source/MoorDyn.h ../../source/Misc.h
	g++ $(EXEFLAGS) -L./ -lmoordyn -I../../source/ -o test_minimal ../../test/minimal.cpp

test: test_minimal
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/ ./test_minimal

debug: CFLAGS += -g -DDEBUG=1
debug: EXEFLAGS += -g -DDEBUG=1

debug: all test

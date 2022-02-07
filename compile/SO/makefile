# makefile for MoorDyn compiled as libmoordyn.so.2.0.0, and creating a convenient symbolic
# link as libmoordyn.so (so it can be linked with -lmoordyn)
#
# The following targets are provided:
#
# all: Builds the library libmoordyn.so.2.0.0, and the symbolic link
#      libmoordyn.so -> libmoordyn.so.2.0.0
#
# test: Build libmoordyn.so, if not yet built, and then the tests:
#        - test_minimal
#
# debug: Build both targets, all and test, with debug information


LFLAGS = -shared -static-libgcc -static-libstdc++ -DMoorDyn_EXPORTS -DLINUX -fPIC
CFLAGS = -c -O3 -g -Wall -Wextra -DLINUX -fPIC -DMoorDyn_EXPORTS

ifdef USEGL
CFLAGS += -DUSEGL
LFLAGS += -lopengl32 -lglu32
endif

CPPFLAGS = $(CFLAGS) -std=c++14
EXEFLAGS = -O3 -g -Wall -Wextra -DLINUX

all: libmoordyn.so

libmoordyn.so: libmoordyn.so.2.0.0
	rm -f libmoordyn.so
	ln -s libmoordyn.so.2.0.0 libmoordyn.so

libmoordyn.so.2.0.0: MoorDyn.o Line.o Connection.o Body.o Rod.o Waves.o Misc.o kiss_fft.o kiss_fftr.o
	$(CXX) $(LFLAGS) -o libmoordyn.so.2.0.0 MoorDyn.o Line.o Connection.o Body.o Rod.o Waves.o Misc.o kiss_fft.o kiss_fftr.o

MoorDyn.o: ../../source/MoorDyn.cpp ../../source/MoorDyn.h ../../source/MoorDynAPI.h
	$(CXX) $(CPPFLAGS) ../../source/MoorDyn.cpp

kiss_fft.o: ../../source/kiss_fft.c ../../source/kiss_fft.h
	$(CXX) $(CFLAGS) ../../source/kiss_fft.c

kiss_fftr.o: ../../source/kiss_fftr.c ../../source/kiss_fftr.h
	$(CXX) $(CFLAGS) ../../source/kiss_fftr.c

Line.o: ../../source/Line.cpp ../../source/Line.h ../../source/QSlines.h
	$(CXX) $(CPPFLAGS) ../../source/Line.cpp

Connection.o: ../../source/Connection.cpp ../../source/Connection.h
	$(CXX) $(CPPFLAGS) ../../source/Connection.cpp

Body.o: ../../source/Body.cpp ../../source/Body.h
	$(CXX) $(CPPFLAGS) ../../source/Body.cpp

Rod.o: ../../source/Rod.cpp ../../source/Rod.h
	$(CXX) $(CPPFLAGS) ../../source/Rod.cpp

Waves.o: ../../source/Waves.cpp ../../source/Waves.h
	$(CXX) $(CPPFLAGS) ../../source/Waves.cpp

Misc.o: ../../source/Misc.cpp ../../source/Misc.h ../../source/MoorDynAPI.h
	$(CXX) $(CPPFLAGS) ../../source/Misc.cpp

clean:
	rm -f *.o *.so *.so.*
	rm -f test_*

test_minimal: ../../test/minimal.cpp libmoordyn.so ../../source/MoorDyn.h
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn -I../../source/ -o test_minimal ../../test/minimal.cpp

test_quasi_static_chain: ../../test/quasi_static_chain.cpp libmoordyn.so ../../source/MoorDyn.h
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn -I../../source/ -o test_quasi_static_chain ../../test/quasi_static_chain.cpp

test: test_minimal test_quasi_static_chain
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/ ./test_minimal
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/ ./test_quasi_static_chain

debug: CFLAGS += -g -DDEBUG=1
debug: CPPFLAGS += -g -DDEBUG=1
debug: EXEFLAGS += -g -DDEBUG=1

debug: all test

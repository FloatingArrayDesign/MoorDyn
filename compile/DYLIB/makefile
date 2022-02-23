# makefile for MoorDyn compiled as libmoordyn2.dylib (so it can be linked with
# -lmoordyn2)
#
# The following targets are provided:
#
# all: Builds the library libmoordyn.dylib
#
# test: Build libmoordyn.dylib, if not yet built, and then the tests:
#        - test_minimal
#        - test_quasi_static_chain
#
# debug: Build both targets, "all" and "test", with debug information


LFLAGS = -shared -static-libgcc -static-libstdc++ -DOSX -DMoorDyn_EXPORTS -fPIC
CFLAGS = -c -O3 -g -Wall -Wextra -DOSX -DMoorDyn_EXPORTS -fPIC

ifdef USEGL
CFLAGS += -DUSEGL
LFLAGS += -lopengl32 -lglu32
endif

CPPFLAGS = $(CFLAGS) -std=c++14
EXEFLAGS = -O3 -g -Wall -Wextra -DOSX

all: libmoordyn2.dylib

libmoordyn2.dylib: MoorDyn.o MoorDyn2.o Line.o Connection.o Body.o Rod.o Waves.o Log.o Misc.o kiss_fft.o kiss_fftr.o
	$(CXX) $(LFLAGS) -o libmoordyn2.dylib MoorDyn.o MoorDyn2.o Line.o Connection.o Body.o Rod.o Waves.o Log.o Misc.o kiss_fft.o kiss_fftr.o

MoorDyn.o: ../../source/MoorDyn.cpp ../../source/MoorDyn.h ../../source/MoorDynAPI.h
	$(CXX) $(CPPFLAGS) ../../source/MoorDyn.cpp

MoorDyn2.o: ../../source/MoorDyn2.cpp ../../source/MoorDyn2.h ../../source/MoorDyn2.hpp ../../source/MoorDynAPI.h
	$(CXX) $(CPPFLAGS) ../../source/MoorDyn2.cpp

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

Log.o: ../../source/Log.cpp ../../source/Log.hpp
	$(CXX) $(CPPFLAGS) ../../source/Log.cpp

Misc.o: ../../source/Misc.cpp ../../source/Misc.h ../../source/MoorDynAPI.h
	$(CXX) $(CPPFLAGS) ../../source/Misc.cpp

clean:
	rm -f *.o *.dylib *.dylib.*
	rm -f test_*

test_minimal: ../../test/minimal.cpp libmoordyn2.dylib ../../source/MoorDyn.h
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn2 -I../../source/ -o test_minimal ../../test/minimal.cpp

test_bodies_and_rods: ../../test/bodies_and_rods.cpp libmoordyn2.dylib ../../source/MoorDyn.h
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn2 -I../../source/ -o test_bodies_and_rods ../../test/bodies_and_rods.cpp

test_quasi_static_chain: ../../test/quasi_static_chain.cpp libmoordyn2.dylib ../../source/MoorDyn.h
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn2 -I../../source/ -o test_quasi_static_chain ../../test/quasi_static_chain.cpp

test: test_minimal test_bodies_and_rods test_quasi_static_chain
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/ ./test_minimal
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/ ./test_bodies_and_rods
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/ ./test_quasi_static_chain

debug: CFLAGS += -O0 -g -DDEBUG=1
debug: CPPFLAGS += -O0 -g -DDEBUG=1
debug: EXEFLAGS += -O0 -g -DDEBUG=1

debug: all test

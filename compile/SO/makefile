# makefile for MoorDyn compiled as libmoordyn.so.2.0.0, and linked as
# libmoordyn.so (so it can be linked with -lmoordyn)
#
# Several options can be set:
#
#  - USEGL: Enables the OpenGL visualization
#  - COPTS: Compilation options. For good performance and compatibility use:
#           COPTS='-msse2 -ffast-math -DMOORDYN_SINGLEPRECISSION'
#           However, for local-only installations a more aggresive instructions
#           set can be used:
#           COPTS='-march=native -ffast-math -DMOORDYN_SINGLEPRECISSION'
#
# The following targets are provided:
#
# all: Builds the library libmoordyn.so.2.0.0, and the symbolic link
#      libmoordyn.so -> libmoordyn.so.2.0.0
#
# test: Build libmoordyn.so, if not yet built, and then the tests:
#        - test_minimal
#        - test_quasi_static_chain
#
# debug: Build both targets, "all" and "test", with debug information


LFLAGS = -shared -static-libgcc -static-libstdc++ -DLINUX -DMoorDyn_EXPORTS -fPIC
CFLAGS = -c -O3 -g -Wall -Wextra -DLINUX -DMoorDyn_EXPORTS -fPIC

ifdef USEGL
CFLAGS += -DUSEGL
LFLAGS += -lopengl32 -lglu32
endif

CFLAGS += $(COPTS)
LFLAGS += $(LOPTS)

CPPFLAGS = $(CFLAGS) -std=c++14
EXEFLAGS = -O3 -g -Wall -Wextra -DLINUX $(EXEOPTS)
HEADERS = ../../source/Body.h \
          ../../source/Connection.h \
          ../../source/_kiss_fft_guts.h \
          ../../source/kiss_fft.h \
          ../../source/kiss_fftr.h \
          ../../source/Line.h \
          ../../source/Misc.h \
          ../../source/MoorDyn2.h \
          ../../source/MoorDynAPI.h \
          ../../source/Rod.h \
          ../../source/Waves.h \
          ../../source/Body.hpp \
          ../../source/Connection.hpp \
          ../../source/Line.hpp \
          ../../source/Log.hpp \
          ../../source/MoorDyn2.hpp \
          ../../source/QSlines.hpp \
          ../../source/Waves.hpp

all: libmoordyn.so

libmoordyn.so: libmoordyn.so.2.0.0
	rm -f libmoordyn.so
	ln -s libmoordyn.so.2.0.0 libmoordyn.so

libmoordyn.so.2.0.0: MoorDyn.o MoorDyn2.o Line.o Connection.o Body.o Rod.o Waves.o Log.o Misc.o kiss_fft.o kiss_fftr.o
	$(CXX) $(LFLAGS) -o libmoordyn.so.2.0.0 MoorDyn.o MoorDyn2.o Line.o Connection.o Body.o Rod.o Waves.o Log.o Misc.o kiss_fft.o kiss_fftr.o

MoorDyn.o: ../../source/MoorDyn.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) ../../source/MoorDyn.cpp

MoorDyn2.o: ../../source/MoorDyn2.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) ../../source/MoorDyn2.cpp

kiss_fft.o: ../../source/kiss_fft.c ../../source/kiss_fft.h ../../source/_kiss_fft_guts.h
	$(CC) $(CFLAGS) ../../source/kiss_fft.c

kiss_fftr.o: ../../source/kiss_fftr.c ../../source/kiss_fftr.h ../../source/_kiss_fft_guts.h
	$(CC) $(CFLAGS) ../../source/kiss_fftr.c

Line.o: ../../source/Line.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) ../../source/Line.cpp

Connection.o: ../../source/Connection.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) ../../source/Connection.cpp

Body.o: ../../source/Body.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) ../../source/Body.cpp

Rod.o: ../../source/Rod.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) ../../source/Rod.cpp

Waves.o: ../../source/Waves.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) ../../source/Waves.cpp

Log.o: ../../source/Log.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) ../../source/Log.cpp

Misc.o: ../../source/Misc.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) ../../source/Misc.cpp

clean:
	rm -f *.o *.so *.so.*
	rm -f test_*

test_minimal: ../../test/minimal.cpp libmoordyn.so
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn -I../../source/ -o test_minimal ../../test/minimal.cpp

test_bodies_and_rods: ../../test/bodies_and_rods.cpp libmoordyn.so
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn -I../../source/ -o test_bodies_and_rods ../../test/bodies_and_rods.cpp

test_wavekin: ../../test/wavekin.cpp libmoordyn.so
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn -I../../source/ -o test_wavekin ../../test/wavekin.cpp

test_quasi_static_chain: ../../test/quasi_static_chain.cpp libmoordyn.so
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn -I../../source/ -o test_quasi_static_chain ../../test/quasi_static_chain.cpp

test: test_minimal test_bodies_and_rods test_wavekin test_quasi_static_chain
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/ ./test_minimal
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/ ./test_bodies_and_rods
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/ ./test_wavekin
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/ ./test_quasi_static_chain

debug: CFLAGS += -O0 -g -DDEBUG=1
debug: CPPFLAGS += -O0 -g -DDEBUG=1
debug: EXEFLAGS += -O0 -g -DDEBUG=1

debug: all test

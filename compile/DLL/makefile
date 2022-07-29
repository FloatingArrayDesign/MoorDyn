# makefile for MoorDyn compiled as libmoordyn2.dll (so it can be linked with
# -lmoordyn2)
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
# all: Builds the library libmoordyn2.dll
#
# test: Build libmoordyn2.dll, if not yet built, and then the tests:
#        - test_minimal
#        - test_quasi_static_chain
#
# debug: Build both targets, "all" and "test", with debug information


LFLAGS = -shared -static-libgcc -static-libstdc++ -DMoorDyn_EXPORTS -fPIC
CFLAGS = -c -O3 -g -Wall -Wextra -DMoorDyn_EXPORTS -fPIC

ifdef USEGL
CFLAGS += -DUSEGL
LFLAGS += -lopengl32 -lglu32
endif

CFLAGS += $(COPTS)
LFLAGS += $(LOPTS)

CPPFLAGS = $(CFLAGS) -std=c++14
EXEFLAGS = -O3 -g -Wall -Wextra -std=c++14 $(EXEOPTS)
HEADERS = ../../source/Body.h \
          ../../source/Connection.h \
          ../../source/_kiss_fft_guts.h \
          ../../source/kiss_fft.h \
          ../../source/kiss_fftr.h \
          ../../source/Line.h \
          ../../source/MoorDyn2.h \
          ../../source/MoorDynAPI.h \
          ../../source/MoorDyn.h \
          ../../source/Rod.h \
          ../../source/Waves.h \
          ../../source/Body.hpp \
          ../../source/Connection.hpp \
          ../../source/IO.hpp \
          ../../source/Line.hpp \
          ../../source/Log.hpp \
          ../../source/Misc.hpp \
          ../../source/MoorDyn2.hpp \
          ../../source/QSlines.hpp \
          ../../source/Rod.hpp \
          ../../source/State.hpp \
          ../../source/Time.hpp \
          ../../source/Waves.hpp

all: libmoordyn2.dll

libmoordyn2.dll: MoorDyn.o MoorDyn2.o Line.o Connection.o Rod.o Body.o Log.o IO.o Misc.o State.o Time.o Waves.o kiss_fftr.o kiss_fft.o
	$(CXX) $(LFLAGS) -o libmoordyn2.dll MoorDyn.o MoorDyn2.o Line.o Rod.o Body.o Connection.o Log.o IO.o Misc.o State.o Time.o Waves.o kiss_fftr.o kiss_fft.o

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

State.o: ../../source/State.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) ../../source/State.cpp

Time.o: ../../source/Time.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) ../../source/Time.cpp

Waves.o: ../../source/Waves.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) ../../source/Waves.cpp

Log.o: ../../source/Log.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) ../../source/Log.cpp

IO.o: ../../source/IO.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) ../../source/IO.cpp

Misc.o: ../../source/Misc.cpp $(HEADERS)
	$(CXX) $(CPPFLAGS) ../../source/Misc.cpp

clean:
	$(RM) *.exe.manifest *.exe *.obj *.o *.dll

test_minimal.exe: ../../test/minimal.cpp libmoordyn2.dll
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn2 -I../../source/ -o test_minimal.exe ../../test/minimal.cpp

test_bodies_and_rods.exe: ../../test/bodies_and_rods.cpp libmoordyn2.dll
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn2 -I../../source/ -o test_bodies_and_rods.exe ../../test/bodies_and_rods.cpp

test_wavekin.exe: ../../test/wavekin.cpp libmoordyn2.dll
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn2 -I../../source/ -o test_wavekin.exe ../../test/wavekin.cpp

test_quasi_static_chain.exe: ../../test/quasi_static_chain.cpp libmoordyn2.dll
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn2 -I../../source/ -o test_quasi_static_chain.exe ../../test/quasi_static_chain.cpp

test: test_minimal.exe test_bodies_and_rods.exe test_quasi_static_chain.exe
	./test_minimal.exe
	./test_bodies_and_rods.exe
	./test_wavekin.exe
	./test_quasi_static_chain.exe

debug: CFLAGS += -O0 -g -DDEBUG=1
debug: CPPFLAGS += -O0 -g -DDEBUG=1
debug: EXEFLAGS += -O0 -g -DDEBUG=1

debug: all test

# makefile for MoorDyn compiled as libmoordyn2.dll (so it can be linked with
# -lmoordyn2)
#
# Several options can be set:
#
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
CFLAGS = -c -O3 -g -Wall -Wextra -DMoorDyn_EXPORTS -fPIC -I../../source/

# Automagically collect the library version
# This should still work fine with mingw, or even in modern Windows with bash
# I have not tested it anyway
CMAKEROOT := ../../CMakeLists.txt
MOORDYN_MAJOR_VERSION=$(shell cat ${CMAKEROOT} | grep "set(MOORDYN_MAJOR_VERSION" | cut -d " " -f 2 | cut -d ")" -f 1)
MOORDYN_MINOR_VERSION=$(shell cat ${CMAKEROOT} | grep "set(MOORDYN_MINOR_VERSION" | cut -d " " -f 2 | cut -d ")" -f 1)
MOORDYN_PATCH_VERSION=$(shell cat ${CMAKEROOT} | grep "set(MOORDYN_PATCH_VERSION" | cut -d " " -f 2 | cut -d ")" -f 1)
CFLAGS += -DMOORDYN_MAJOR_VERSION=$(MOORDYN_MAJOR_VERSION)
CFLAGS += -DMOORDYN_MINOR_VERSION=$(MOORDYN_MINOR_VERSION)
CFLAGS += -DMOORDYN_PATCH_VERSION=$(MOORDYN_PATCH_VERSION)

CFLAGS += $(COPTS)
LFLAGS += $(LOPTS)

CPPFLAGS = $(CFLAGS) -std=c++17
EXEFLAGS = -O3 -g -Wall -Wextra -std=c++17 $(EXEOPTS)
HEADERS = $(wildcard ../../source/*.hpp) \
          $(wildcard ../../source/Util/*.hpp) \
          $(wildcard ../../source/Waves/*.hpp) \
          $(wildcard ../../source/*.h)
SOURCES = $(wildcard ../../source/*.cpp) \
          $(wildcard ../../source/Util/*.cpp) \
          $(wildcard ../../source/Waves/*.cpp)
OBJECTS = $(patsubst ../../source/%,%,$(patsubst %.cpp,%.o,$(SOURCES)))
DIRGUARD = @mkdir -p $(@D)

all: libmoordyn2.dll

libmoordyn2.dll: $(OBJECTS)
	$(CXX) $(LFLAGS) -o libmoordyn2.dll $(OBJECTS)

%.o: ../../source/%.cpp $(HEADERS)
	${CXX} $(CPPFLAGS) -o $@ $<
Util/%.o: ../../source/Util/%.cpp $(HEADERS)
	$(DIRGUARD)
	${CXX} $(CPPFLAGS) -o $@ $<
Waves/%.o: ../../source/Waves/%.cpp $(HEADERS)
	$(DIRGUARD)
	${CXX} $(CPPFLAGS) -o $@ $<

clean:
	$(RM) *.exe.manifest *.exe *.obj *.o ./*/*.o *.dll

test_minimal.exe: ../../tests/minimal.cpp libmoordyn2.dll
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn2 -I../../source/ -o test_minimal.exe ../../tests/minimal.cpp

test_bodies_and_rods.exe: ../../tests/bodies_and_rods.cpp libmoordyn2.dll
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn2 -I../../source/ -o test_bodies_and_rods.exe ../../tests/bodies_and_rods.cpp

test_wavekin.exe: ../../tests/wavekin.cpp libmoordyn2.dll
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn2 -I../../source/ -o test_wavekin.exe ../../tests/wavekin.cpp

test_quasi_static_chain.exe: ../../tests/quasi_static_chain.cpp libmoordyn2.dll
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn2 -I../../source/ -o test_quasi_static_chain.exe ../../tests/quasi_static_chain.cpp

test: test_minimal.exe test_bodies_and_rods.exe test_quasi_static_chain.exe
	./test_minimal.exe
	./test_bodies_and_rods.exe
	./test_wavekin.exe
	./test_quasi_static_chain.exe

debug: CFLAGS += -O0 -g -DDEBUG=1
debug: CPPFLAGS += -O0 -g -DDEBUG=1
debug: EXEFLAGS += -O0 -g -DDEBUG=1

debug: all test

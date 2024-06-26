# makefile for MoorDyn compiled as libmoordyn2.dylib (so it can be linked with
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
# all: Builds the library libmoordyn.dylib
#
# test: Build libmoordyn.dylib, if not yet built, and then the tests:
#        - test_minimal
#        - test_quasi_static_chain
#
# debug: Build both targets, "all" and "test", with debug information


# # Flags for compiling with gcc (not OSX default)
# LFLAGS = -shared -static-libgcc -static-libstdc++ -DOSX -DMoorDyn_EXPORTS -fPIC
# CFLAGS = -c -O3 -g -Wall -Wextra -DOSX -DMoorDyn_EXPORTS -fPIC

# Flags for compiling with clang (OSX default)
LFLAGS = -shared -DOSX -DMoorDyn_EXPORTS -fPIC
CFLAGS = -c -O3 -static -g -Wall -Wextra -DOSX -DMoorDyn_EXPORTS -fPIC -I../../source/

# Automagically collect the library version
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
EXEFLAGS = -O3 -g -Wall -Wextra -DOSX $(EXEOPTS)

CC = clang++
CXX = clang++

HEADERS = $(wildcard ../../source/*.hpp) \
          $(wildcard ../../source/Util/*.hpp) \
          $(wildcard ../../source/Waves/*.hpp) \
          $(wildcard ../../source/*.h)
SOURCES = $(wildcard ../../source/*.cpp) \
          $(wildcard ../../source/Util/*.cpp) \
          $(wildcard ../../source/Waves/*.cpp)
OBJECTS = $(patsubst ../../source/%,%,$(patsubst %.cpp,%.o,$(SOURCES)))
DIRGUARD = @mkdir -p $(@D)

all: libmoordyn2.dylib

libmoordyn2.dylib: $(OBJECTS)
	$(CXX) $(LFLAGS) -o libmoordyn2.dylib $(OBJECTS)

%.o: ../../source/%.cpp $(HEADERS)
	${CXX} $(CPPFLAGS) -o $@ $<
Util/%.o: ../../source/Util/%.cpp $(HEADERS)
	$(DIRGUARD)
	${CXX} $(CPPFLAGS) -o $@ $<
Waves/%.o: ../../source/Waves/%.cpp $(HEADERS)
	$(DIRGUARD)
	${CXX} $(CPPFLAGS) -o $@ $<

clean:
	rm -f *.o ./*/*.o *.dylib *.dylib.*
	rm -f test_*

test_minimal: ../../tests/minimal.cpp libmoordyn2.dylib
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn2 -I../../source/ -o test_minimal ../../tests/minimal.cpp

test_bodies_and_rods: ../../tests/bodies_and_rods.cpp libmoordyn2.dylib
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn2 -I../../source/ -o test_bodies_and_rods ../../tests/bodies_and_rods.cpp

test_wavekin: ../../tests/wavekin.cpp libmoordyn2.dylib
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn2 -I../../source/ -o test_wavekin ../../tests/wavekin.cpp

test_quasi_static_chain: ../../tests/quasi_static_chain.cpp libmoordyn2.dylib
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn2 -I../../source/ -o test_quasi_static_chain ../../tests/quasi_static_chain.cpp

test: test_minimal test_bodies_and_rods test_quasi_static_chain
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/ ./test_minimal
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/ ./test_bodies_and_rods
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/ ./test_wavekin
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/ ./test_quasi_static_chain

debug: CFLAGS += -O0 -g -DDEBUG=1
debug: CPPFLAGS += -O0 -g -DDEBUG=1
debug: EXEFLAGS += -O0 -g -DDEBUG=1

debug: all test

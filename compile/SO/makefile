# makefile for MoorDyn compiled as libmoordyn.so.2.0.0, and linked as
# libmoordyn.so (so it can be linked with -lmoordyn)
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
# all: Builds the library libmoordyn.so.2.0.0, and the symbolic link
#      libmoordyn.so -> libmoordyn.so.2.0.0
#
# test: Build libmoordyn.so, if not yet built, and then the tests:
#        - test_minimal
#        - test_quasi_static_chain
#
# debug: Build both targets, "all" and "test", with debug information


LFLAGS = -shared -static-libgcc -static-libstdc++ -DLINUX -DMoorDyn_EXPORTS -fPIC
CFLAGS = -c -O3 -g -Wall -Wextra -DLINUX -DMoorDyn_EXPORTS -fPIC -I../../source/

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
EXEFLAGS = -O3 -g -Wall -Wextra -DLINUX $(EXEOPTS)
HEADERS = $(wildcard ../../source/*.hpp) \
          $(wildcard ../../source/Util/*.hpp) \
          $(wildcard ../../source/Waves/*.hpp) \
          $(wildcard ../../source/*.h)
SOURCES = $(wildcard ../../source/*.cpp) \
          $(wildcard ../../source/Util/*.cpp) \
          $(wildcard ../../source/Waves/*.cpp)
OBJECTS = $(patsubst ../../source/%,%,$(patsubst %.cpp,%.o,$(SOURCES)))
DIRGUARD = @mkdir -p $(@D)

all: libmoordyn.so

libmoordyn.so: libmoordyn.so.$(MOORDYN_MAJOR_VERSION).$(MOORDYN_MINOR_VERSION)
	rm -f libmoordyn.so
	ln -s libmoordyn.so.$(MOORDYN_MAJOR_VERSION).$(MOORDYN_MINOR_VERSION) libmoordyn.so

libmoordyn.so.$(MOORDYN_MAJOR_VERSION).$(MOORDYN_MINOR_VERSION): libmoordyn.so.$(MOORDYN_MAJOR_VERSION).$(MOORDYN_MINOR_VERSION).$(MOORDYN_PATCH_VERSION)
	rm -f libmoordyn.so
	ln -s libmoordyn.so.$(MOORDYN_MAJOR_VERSION).$(MOORDYN_MINOR_VERSION).$(MOORDYN_PATCH_VERSION) libmoordyn.so.$(MOORDYN_MAJOR_VERSION).$(MOORDYN_MINOR_VERSION)

libmoordyn.so.$(MOORDYN_MAJOR_VERSION).$(MOORDYN_MINOR_VERSION).$(MOORDYN_PATCH_VERSION): $(OBJECTS)
	$(CXX) $(LFLAGS) -o libmoordyn.so.$(MOORDYN_MAJOR_VERSION).$(MOORDYN_MINOR_VERSION).$(MOORDYN_PATCH_VERSION) $(OBJECTS)

%.o: ../../source/%.cpp $(HEADERS)
	${CXX} $(CPPFLAGS) -o $@ $<
Util/%.o: ../../source/Util/%.cpp $(HEADERS)
	$(DIRGUARD)
	${CXX} $(CPPFLAGS) -o $@ $<
Waves/%.o: ../../source/Waves/%.cpp $(HEADERS)
	$(DIRGUARD)
	${CXX} $(CPPFLAGS) -o $@ $<

clean:
	rm -f *.o ./*/*.o *.so *.so.*
	rm -f test_*

test_minimal: ../../tests/minimal.cpp libmoordyn.so
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn -I../../source/ -o test_minimal ../../tests/minimal.cpp

test_bodies_and_rods: ../../tests/bodies_and_rods.cpp libmoordyn.so
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn -I../../source/ -o test_bodies_and_rods ../../tests/bodies_and_rods.cpp

test_wavekin: ../../tests/wavekin.cpp libmoordyn.so
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn -I../../source/ -o test_wavekin ../../tests/wavekin.cpp

test_quasi_static_chain: ../../tests/quasi_static_chain.cpp libmoordyn.so
	$(CXX) $(EXEFLAGS) -L./ -lmoordyn -I../../source/ -o test_quasi_static_chain ../../tests/quasi_static_chain.cpp

test: test_minimal test_bodies_and_rods test_wavekin test_quasi_static_chain
	cd ../../tests/; \
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/../compile/SO/ ../compile/SO/test_minimal
	cd ../../tests/; \
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/../compile/SO/ ../compile/SO/test_bodies_and_rods
	cd ../../tests/; \
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/../compile/SO/ ../compile/SO/test_wavekin
	cd ../../tests/; \
	LD_LIBRARY_PATH=$LD_LIBRARY_PATH:`pwd`/../compile/SO/ ../compile/SO/test_wavekin

debug: CFLAGS += -O0 -g -DDEBUG=1
debug: CPPFLAGS += -O0 -g -DDEBUG=1
debug: EXEFLAGS += -O0 -g -DDEBUG=1

debug: all test

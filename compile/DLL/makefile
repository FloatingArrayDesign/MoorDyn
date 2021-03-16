# makefile for MoorDyn v2 compiled as MoorDyn2.dll
# optional compile def:  USEGL

# tell to search for dependencies in parent dir
VPATH = ../../source
LFLAGS = -shared -static -static-libgcc -static-libstdc++ -DMoorDyn_EXPORTS


#CFLAGS = -c -O0 -g -w -Wall -static -static-libgcc -static-libstdc++ -std=c++17 -DMoorDyn_EXPORTS
#CFLAGS = -c -O0 -g -w -Wall -static -static-libgcc -static-libstdc++ -std=gnu++0x -DMoorDyn_EXPORTS
#CFLAGS = -c -Og -ggdb -w -Wall -static -static-libgcc -static-libstdc++ -std=gnu++0x -DMoorDyn_EXPORTS

# this is the optimized one
CFLAGS = -c -O3 -g -w -Wall -static -static-libgcc -static-libstdc++ -std=gnu++0x -DMoorDyn_EXPORTS


all: MoorDyn2.dll
 
MoorDyn2.dll: MoorDyn.o Line.o Connection.o Rod.o Body.o Misc.o Waves.o kiss_fftr.o kiss_fft.o
	g++ $(LFLAGS) -o MoorDyn2.dll MoorDyn.o Line.o Rod.o Body.o Connection.o Misc.o Waves.o kiss_fftr.o kiss_fft.o -lopengl32 -lglu32
# added linking to openGL above

MoorDyn.o: MoorDyn.cpp MoorDyn.h Line.h Line.cpp Rod.h Rod.cpp Body.h Body.cpp Connection.h Connection.cpp QSlines.h Waves.h Waves.cpp Misc.h Misc.cpp
	g++ $(CFLAGS) ../../source/MoorDyn.cpp
	
kiss_fftr.o: kiss_fftr.h kiss_fftr.c kiss_fft.h
	g++ $(CFLAGS) ../../source/kiss_fftr.c
	
kiss_fft.o: kiss_fft.h kiss_fft.c 
	g++ $(CFLAGS) ../../source/kiss_fft.c
	
Waves.o: Waves.h Waves.cpp
	g++ $(CFLAGS) ../../source/Waves.cpp
	
Line.o: Line.h Line.cpp Connection.h Connection.cpp QSlines.h Waves.h Misc.h
	g++ $(CFLAGS) ../../source/Line.cpp
	
Rod.o: Rod.h Rod.cpp Line.h Line.cpp Connection.h Connection.cpp Waves.h Misc.h
	g++ $(CFLAGS) ../../source/Rod.cpp

Body.o: Line.h Line.cpp Connection.h Rod.h Body.h Body.cpp Waves.h Misc.h Misc.cpp
	g++ $(CFLAGS) ../../source/Body.cpp	
	
Connection.o: Line.h Line.cpp Connection.h Connection.cpp Waves.h Misc.h Misc.cpp
	g++ $(CFLAGS) ../../source/Connection.cpp
	
Misc.o: Misc.h Misc.cpp
	g++ $(CFLAGS) ../../source/Misc.cpp

clean:
	del *.exe.manifest *.exe *.obj *.o *.dll


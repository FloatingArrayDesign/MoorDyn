# makefile for MoorDyn compiled as Lines.dll


# tell to search for dependencies in parent dir
VPATH = ../

CFLAGS2 = -shared -static -static-libgcc -static-libstdc++ -DMoorDyn_EXPORTS

CFLAGS = -c -g -w -Wall -static -static-libgcc -static-libstdc++ -std=gnu++0x -DMoorDyn_EXPORTS
# removed -O3 for debugging

#CFLAGS2 = -shared -static-libgcc 
#CFLAGS = -c -g -w -Wall -static-libgcc -std=gnu++0x -DMoorDyn_EXPORTS

# excess of "static" flags above is to avoid DLL dependencies (it probably needs cleanup)

all: Lines.dll
 
Lines.dll: main.o Line.o Connection.o Misc.o kiss_fft.o
	g++ $(CFLAGS2) -o Lines.dll main.o Line.o Connection.o Misc.o kiss_fft.o

main.o: main.cpp main.h Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h Misc.cpp
	g++ $(CFLAGS) ../main.cpp
	
kiss_fft.o: kiss_fft.h kiss_fft.c
	g++ $(CFLAGS) ../kiss_fft.c
	
Line.o: Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h
	g++ $(CFLAGS) ../Line.cpp

Connection.o: Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h Misc.cpp
	g++ $(CFLAGS) ../Connection.cpp
	
Misc.o: Misc.h Misc.cpp
	g++ $(CFLAGS) ../Misc.cpp

clean:
	del *.exe.manifest *.exe *.obj *.o *.dll


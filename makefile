# makefile for MoorDyn compiled as Lines.dll

CFLAGS2 = -shared -static -static-libgcc -static-libstdc++

CFLAGS = -c -O3 -g -w -Wall -static -static-libgcc -static-libstdc++ -std=gnu++0x -DMoorDyn_EXPORTS

# excess of "static" flags above is to avoid DLL dependencies (it probably needs cleanup)

all: Lines.dll
 
Lines.dll: main.o Line.o Connection.o Misc.o
	g++ $(CFLAGS2) -o Lines.dll main.o Line.o Connection.o Misc.o

main.o: main.cpp main.h Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h Misc.cpp
	g++ $(CFLAGS) main.cpp
	
Line.o: Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h
	g++ $(CFLAGS) Line.cpp

Connection.o: Line.h Line.cpp Connection.h Connection.cpp QSlines.h Misc.h Misc.cpp
	g++ $(CFLAGS) Connection.cpp
	
Misc.o: Misc.h Misc.cpp
	g++ $(CFLAGS) Misc.cpp

clean:
	del *.exe.manifest *.exe *.obj *.o *.dll

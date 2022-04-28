#!/bin/sh

COPTS=''

for arg in "$@"; do
    if [ $arg = "--optimize" ]; then
        COPTS='-msse2 -ffast-math -DMOORDYN_SINGLEPRECISSION'
    fi
done

CXX=i686-w64-mingw32-g++ CC=i686-w64-mingw32-gcc make clean
CXX=i686-w64-mingw32-g++ CC=i686-w64-mingw32-gcc make COPTS="$COPTS" -j

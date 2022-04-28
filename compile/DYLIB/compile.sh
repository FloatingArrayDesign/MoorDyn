#!/bin/sh

COPTS=''

for arg in "$@"; do
    if [ $arg = "--optimize" ]; then
        COPTS='-msse2 -ffast-math -DMOORDYN_SINGLEPRECISSION'
    fi
done

CXX=clang++ CC=clang make clean
CXX=clang++ CC=clang make COPTS="$COPTS" -j

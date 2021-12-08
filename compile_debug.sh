#!/bin/sh

mkdir -p build/debug
cd build/debug

#make -j4
make

cd ../..
cp build/debug/compile_commands.json .
ctags -R --exclude=.git .


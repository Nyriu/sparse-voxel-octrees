#!/bin/sh

mkdir -p build/release
cd build/release

#make -j4
make

cd ../..
cp build/release/compile_commands.json .
ctags -R --exclude=.git .


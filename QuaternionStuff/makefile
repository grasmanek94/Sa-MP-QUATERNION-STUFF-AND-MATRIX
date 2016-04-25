# This file demonstrates how to compile the plugin project on Linux.
#
# To compile plugin do:
#
# make plugin 
#

GPP = g++
GCC = gcc
plugin_OUTFILE = "../QuaternionStuff.so"

COMPILE_FLAGS = -std=c++0x -m32 -O3 -fPIC -c -I ./ -w -D LINUX -D PROJECT_NAME=\"plugin\"

plugin = -D plugin $(COMPILE_FLAGS)

all: plugin

clean:
	-rm -f *~ *.o *.so

plugin: clean
	$(GPP) $(plugin) ./*.cpp
	$(GPP) -std=c++0x -m32 --static -fshort-wchar -shared -o $(plugin_OUTFILE) *.o


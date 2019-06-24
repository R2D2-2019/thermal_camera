#############################################################################
#
# Project Makefile
#
# (c) Wouter van Ooijen (www.voti.nl) 2016
#
# This file is in the public domain.
# 
#############################################################################

# source files in this project (main.cpp is automatically assumed)
SOURCES := $(wildcard src/*.cpp src/dynamic_vars/*.cpp src/lookupables/*.cpp src/pixel_manipulators/*.cpp src/static_vars/*.cpp)

# header files in this project
HEADERS := $(wildcard headers/*.hpp headers/dynamic_vars/*.hpp headers/lookupables/*.hpp headers/pixel_manipulators/*.hpp headers/static_vars/*.hpp)

# other places to look for files for this project
SEARCH  := ./headers ./src 

# set REATIVE to the next higher directory 
# and defer to the Makefile.due there
RELATIVE := $(RELATIVE)../
include $(RELATIVE)Makefile.due


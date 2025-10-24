INC_DIRS = \
    ./include
    # ../path/to/hud-def

LIB_DIRS = \
    ./src/dlfdm
    
SRC_DIR = \
    ./test
    
PROJECT_NAME = dlfdm-test
MAIN_CXX = main

#PROJECT_NAME = dlfdm-hud-test
#MAIN_CXX = hud-data-example

# Compile with debug symbols
#USERCPPFLAGS = -g -Wall

include ./Makefile.master

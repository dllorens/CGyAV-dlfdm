INC_DIRS = \
    ./include

LIB_DIRS = \
    ./src/dlfdm
    
SRC_DIR = \
    ./test
    
PROJECT_NAME = dlfdm-test
MAIN_CXX = main

# Compile with debug symbols
#USERCPPFLAGS = -g -Wall

include ./Makefile.master

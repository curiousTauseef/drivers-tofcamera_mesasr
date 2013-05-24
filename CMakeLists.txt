# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
include($ENV{ROCK_CMAKE_MACROS}/Rock.cmake)

##### Download Swissranger Linux Driver if not present ############################
find_library(MESA_LIBRARY  mesasr)

if (${MESA_LIBRARY} MATCHES "MESA_LIBRARY-NOTFOUND")

    if(NOT EXISTS ${PROJECT_SOURCE_DIR}/external)
        execute_process(COMMAND mkdir ${PROJECT_SOURCE_DIR}/external)
    endif(NOT EXISTS ${PROJECT_SOURCE_DIR}/external)

    ### 32 bit
    if (${CMAKE_SIZEOF_VOID_P} MATCHES 4)
        execute_process(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/external COMMAND wget http://www.mesa-imaging.ch/dlm.php?fname=./customer/driver/libmesasr-dev-1.0.14-706.i386.deb --user-agent="" --output-document=libmesasr-dev-1.0.14-706.i386.deb)
        execute_process(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/external COMMAND sudo dpkg -i libmesasr-dev-1.0.14-706.i386.deb)
    ### 64 bit
    elseif (${CMAKE_SIZEOF_VOID_P} MATCHES 8)
        execute_process(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/external COMMAND wget http://www.mesa-imaging.ch/dlm.php?fname=./customer/driver/libmesasr-dev-1.0.14-706.amd64.deb --user-agent="" --output-document=libmesasr-dev-1.0.14-706.amd64.deb)
        execute_process(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/external COMMAND sudo dpkg -i libmesasr-dev-1.0.14-706.amd64.deb)
    else (${CMAKE_SIZEOF_VOID_P} MATCHES 4)
        message(WARNING "Only 32 and 64 bit is supported by the Mesa Swissranger driver")
    endif (${CMAKE_SIZEOF_VOID_P} MATCHES 4)

endif (${MESA_LIBRARY} MATCHES "MESA_LIBRARY-NOTFOUND")

rock_init(tofcamera_mesa_swissranger 0.1)
rock_standard_layout()

# CMakeLists.txt has to be located in the project folder and cmake has to be
# executed from 'project/build' with 'cmake ../'.
cmake_minimum_required(VERSION 2.6)
find_package(Rock)

##### Find, download and install Swissranger Linux Driver ############################

### Check if the last driver version already installed
### last version for 32 bit -> 1.0.14.747; for 64 bit -> 1.0.14.748
### 
### 32 bit
if (${CMAKE_SIZEOF_VOID_P} MATCHES 4)
    LIST(APPEND CMAKE_FIND_LIBRARY_SUFFIXES ".so.1.0.14.747")
    find_library(MESASR_LIBRARY_LAST  NAMES "libmesasr.so.1.0.14.747")
    LIST(REMOVE_ITEM CMAKE_FIND_LIBRARY_SUFFIXES ".so.1.0.14.747")
### 64 bit
elseif (${CMAKE_SIZEOF_VOID_P} MATCHES 8)
    LIST(APPEND CMAKE_FIND_LIBRARY_SUFFIXES ".so.1.0.14.748")
    find_library(MESASR_LIBRARY_LAST  NAMES "libmesasr.so.1.0.14.748")
    LIST(REMOVE_ITEM CMAKE_FIND_LIBRARY_SUFFIXES ".so.1.0.14.748")
else ()
    message(FATAL_ERROR "The system is unknown.")
endif ()
    
message(STATUS "Last version of MesaSR library: ${MESASR_LIBRARY_LAST}")

### If the last driver version is not installed
if (${MESASR_LIBRARY_LAST} MATCHES "MESASR_LIBRARY_LAST-NOTFOUND")
    ### Check if the any other version already installed
    ### if so, than remove the current driver package
    find_library(MESASR_LIBRARY  "mesasr")
    message(STATUS "The MesaSR library: ${MESASR_LIBRARY}")
    if (NOT ${MESASR_LIBRARY} MATCHES "MESASR_LIBRARY-NOTFOUND")
        ### remove the existing library package
        execute_process(COMMAND sudo dpkg -r libmesasr-dev)
        execute_process(COMMAND sudo dpkg -p libmesasr-dev)
    endif (NOT ${MESASR_LIBRARY} MATCHES "MESASR_LIBRARY-NOTFOUND")

    ### Download the last driver version and install it
    if(NOT EXISTS ${PROJECT_SOURCE_DIR}/external)
        execute_process(COMMAND mkdir ${PROJECT_SOURCE_DIR}/external)
    endif(NOT EXISTS ${PROJECT_SOURCE_DIR}/external)

    ### 32 bit
    if (${CMAKE_SIZEOF_VOID_P} MATCHES 4)
        execute_process(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/external COMMAND wget http://www.mesa-imaging.ch/dlm.php?fname=./customer/driver/libmesasr-dev-1.0.14-747.i386.deb --user-agent="" --output-document=libmesasr-dev-1.0.14-747.i386.deb)
        execute_process(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/external COMMAND sudo dpkg -i libmesasr-dev-1.0.14-747.i386.deb)
        execute_process(COMMAND sudo dpkg --configure -a)
    ### 64 bit
    elseif (${CMAKE_SIZEOF_VOID_P} MATCHES 8)
        execute_process(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/external COMMAND wget http://www.mesa-imaging.ch/dlm.php?fname=./customer/driver/libmesasr-dev-1.0.14-748.amd64.deb --user-agent="" --output-document=libmesasr-dev-1.0.14-748.amd64.deb)
        execute_process(WORKING_DIRECTORY ${PROJECT_SOURCE_DIR}/external COMMAND sudo dpkg -i libmesasr-dev-1.0.14-748.amd64.deb)
        execute_process(COMMAND sudo dpkg --configure -a)
    else ()
        message(FATAL_ERROR "The system is unknown.")
    endif ()

endif ()

rock_init(tofcamera_mesa_swissranger 0.1)
rock_standard_layout()

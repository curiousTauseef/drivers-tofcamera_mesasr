set(CMAKE_VERBOSE_MAKEFILE on)

if (${CMAKE_SIZEOF_VOID_P} MATCHES 4)
   SET(MESA_EXTERNAL ${PROJECT_SOURCE_DIR}/external/libmesasr-dev-1.0.14-706.i386)
elseif (${CMAKE_SIZEOF_VOID_P} MATCHES 8)
   SET(MESA_EXTERNAL ${PROJECT_SOURCE_DIR}/external/libmesasr-dev-1.0.14-706.amd64)
else (${CMAKE_SIZEOF_VOID_P} MATCHES 4)
   message(WARNING "Only 32 and 64 bit is supported by the mesa swissranger api")
endif (${CMAKE_SIZEOF_VOID_P} MATCHES 4)

# MESA 
include_directories(${MESA_EXTERNAL}/include)
install(DIRECTORY ${MESA_EXTERNAL}/include/ DESTINATION include/${PROJECT_NAME}
	FILES_MATCHING PATTERN "*.h")
link_directories(${MESA_EXTERNAL}/lib)

# Library
rock_library(${PROJECT_NAME}
    SOURCES SwissRangerDriver.cpp
    HEADERS SwissRangerDriver.hpp SwissRangeDriverLookUp.hpp SwissRangerTypes.hpp
    DEPS_PKGCONFIG base-types base-lib)
target_link_libraries(${PROJECT_NAME} ${MESA_EXTERNAL}/lib/libmesasr.so)

rock_executable(tofcamera_mesa_swissranger_bin Main.cpp
    DEPS ${PROJECT_NAME})

target_link_libraries(tofcamera_mesa_swissranger_bin ${PROJECT_NAME} ${MESA_EXTERNAL}/lib/libmesasr.so)




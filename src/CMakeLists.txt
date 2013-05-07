SET(module_name ${PROJECT_NAME})

if (${CMAKE_SIZEOF_VOID_P} MATCHES 4)
    include_directories(${PROJECT_SOURCE_DIR}/external/libmesasr-dev-1.0.14-706.i386/include)
    link_directories(${PROJECT_SOURCE_DIR}/external/libmesasr-dev-1.0.14-706.i386/lib)
    get_directory_property(LINK_DIR LINK_DIRECTORIES)
elseif (${CMAKE_SIZEOF_VOID_P} MATCHES 8)
    include_directories(${PROJECT_SOURCE_DIR}/external/libmesasr-dev-1.0.14-706.amd64/include)
    link_directories(${PROJECT_SOURCE_DIR}/external/libmesasr-dev-1.0.14-706.amd64/lib)
    get_directory_property(LINK_DIR LINK_DIRECTORIES)
else (${CMAKE_SIZEOF_VOID_P} MATCHES 4)
    message(WARNING "Only 32 and 64 bit is supported by the prosilica api")
endif (${CMAKE_SIZEOF_VOID_P} MATCHES 4)


rock_library(${PROJECT_NAME}
    SOURCES SwissRangerDriver.cpp
    HEADERS SwissRangerDriver.hpp SwissRangeDriverLookUp.hpp SwissRangerTypes.hpp
    DEPS_PKGCONFIG base-types base-lib)

target_link_libraries(${PROJECT_NAME} ${LINK_DIR}/libmesasr.so.1.0.14)

rock_executable(tofcamera_mesa_swissranger_bin Main.cpp
    DEPS ${PROJECT_NAME})

target_link_libraries(tofcamera_mesa_swissranger_bin ${PROJECT_NAME} ${LINK_DIR}/libmesasr.so.1.0.14)


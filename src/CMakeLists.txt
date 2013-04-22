rock_library(tofcamera_mesa_swissranger
    SOURCES Dummy.cpp
    HEADERS Dummy.hpp)

rock_executable(tofcamera_mesa_swissranger_bin Main.cpp
    DEPS tofcamera_mesa_swissranger)


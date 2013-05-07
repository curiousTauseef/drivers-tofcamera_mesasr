rock_vizkit_plugin(tofcamera_mesa_swissranger-viz
    TOFVisualization.cpp
    DEPS tofcamera_mesa_swissranger
    DEPS_PKGCONFIG envire envire-viz
    MOC TOFVisualization.hpp
    HEADERS TOFVisualization.hpp)


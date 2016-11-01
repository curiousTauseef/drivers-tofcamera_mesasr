# Library
rock_library(${PROJECT_NAME}
    SOURCES SwissRangerDriver.cpp
    HEADERS SwissRangerDriver.hpp SwissRangeDriverLookUp.hpp SwissRangerTypes.hpp
    DEPS_PKGCONFIG 
        base-types 
	base-lib
	base-logging
)
target_link_libraries(${PROJECT_NAME} mesasr)

rock_executable(tofcamera_mesasr_bin Main.cpp
    DEPS ${PROJECT_NAME})
target_link_libraries(tofcamera_mesasr_bin ${PROJECT_NAME} mesasr)





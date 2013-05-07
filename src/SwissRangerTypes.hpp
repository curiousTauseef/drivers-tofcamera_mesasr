#ifndef SWISSRANGERTYPES_HPP
#define SWISSRANGERTYPES_HPP

#include <base/time.h>
#include <vector>

#include "libMesaSR.h"

namespace tofcamera_mesa_swissranger
{

    typedef enum ModulationFrq TMS_ModulationFrq;

    enum TMS_CoordPercision {
        CP_UINT = 0,
        CP_FLOAT = 1,
        CP_DOUBLE = 2
    };

//    /**
//     * @brief used to reperesent distance, amplitude and confidence map data
//     * holds a width and height of the image
//     * the data and the saturation values
//     * \note row major
//     */
//    struct TOFImage
//    {
//            base::Time time;

//            unsigned int width;
//            unsigned int height;

//            /**
//             * @brief data
//             * \note value above 32767 indicates saturation.
//             */
//            std::vector<int> data;
//    };

} // end namespace tofcamera_mesa_swissranger

#endif // SWISSRANGERTYPES_HPP

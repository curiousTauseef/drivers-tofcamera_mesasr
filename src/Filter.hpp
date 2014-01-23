#ifndef FILTER_HPP
#define FILTER_HPP           

#include <tofcamera_mesa_swissranger/TOFScan.hpp>
#include <vector> 

namespace tofcamera_mesa_swissranger
{

    class Filter 
    {
    public:
        static void filteConfidence(tofcamera_mesa_swissranger::TOFScan &tofscan ,unsigned int min_confidence)
        {
            std::vector<uint16_t> *confidence_image = (std::vector<uint16_t>*)&(tofscan.confidence_image);

            // delete all points in pointcloud which confidence is lower than minimal allowed confidence value
            for (unsigned int i = 0; i < confidence_image->size(); ++i)
            {
                if (confidence_image->at(i) < min_confidence)
                {
                    tofscan.coordinates_3D.at(i).x() = 0;
                    tofscan.coordinates_3D.at(i).y() = 0;
                    tofscan.coordinates_3D.at(i).z() = 0;
                }
            }
        }

        static void filteAmplitude(tofcamera_mesa_swissranger::TOFScan &tofscan,
                                    unsigned int min_amplitude,
                                    unsigned int max_amplitude)
        {
            std::vector<uint16_t> *amplitude_image = (std::vector<uint16_t>*)&(tofscan.amplitude_image);

            // delete all points in pointcloud which confidence is lower than minimal allowed confidence value
            for (unsigned int i = 0; i < amplitude_image->size(); ++i)
            {
                if (amplitude_image->at(i) < min_amplitude || amplitude_image->at(i) > max_amplitude)
                {
                    tofscan.coordinates_3D.at(i).x() = 0;
                    tofscan.coordinates_3D.at(i).y() = 0;
                    tofscan.coordinates_3D.at(i).z() = 0;
                }
            }
        }
    };

}

#endif // FILTER_HPP            
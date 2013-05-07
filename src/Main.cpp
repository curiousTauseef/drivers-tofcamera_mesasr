#include <iostream>
#include <base/logging.h>
#include "SwissRangerDriver.hpp"
#include "SwissRangeDriverLookUp.hpp"

using namespace tofcamera_mesa_swissranger;

void printPointCloud(base::samples::Pointcloud &pointcloud);
void printPointCloud(base::samples::Pointcloud &pointcloud_1,
                     base::samples::Pointcloud &pointcloud_2,
                     base::samples::Pointcloud &pointcloud_3);

int main(int argc, char** argv)
{
    SwissRangerDriver driver;

    bool result = false;

    // open device over USB
    if (driver.openUSB(0) == false) {
        LOG_ERROR("openUSB: error");
        return false;
    }

    // ----------------- TEST: the properties ---------------

    // --- Library Version
    std::string version;
    result = driver.getLibraryVersion(version);
    if (result == true)
        LOG_INFO("getVersionOfLibrary: %s", version.c_str());

    // --- Device Info
    std::string device_info;
    result = driver.getDeviceInfo(device_info);
    if (result == true)
        LOG_INFO("getDeviceInfo: %s", device_info.c_str());

    // --- Acquisition Mode
    int acquire_mode = AM_COR_FIX_PTRN;
    result = driver.getAcquisitionMode(acquire_mode);
    if (result == true) {
        std::string modes_str;
        acquireModesToStr(acquire_mode, modes_str);
        LOG_INFO("getAcquisitionMode: %s", modes_str.c_str());
    }

    acquire_mode = AM_COR_FIX_PTRN|AM_CONV_GRAY|AM_DENOISE_ANF|AM_CONF_MAP;
    result = driver.setAcquisitionMode(acquire_mode);
    if (result == true)
        LOG_INFO("setAcquisitionMode: success");

    acquire_mode = AM_COR_FIX_PTRN;
    result = driver.turnAcquisitionMode(AM_COR_FIX_PTRN, false);
    if (result == true)
        LOG_INFO("turnAcquisitionMode: success");

    // --- Timeout
    driver.setTimeout(4000);
    int timeout = driver.getTimeout();
    LOG_INFO("setTimeout: %d", timeout);

    // --- Rows and Cols
    unsigned int rows = 0;
    result = driver.getRows(rows);
    if (result == true)
        LOG_INFO("getRows: %d", rows);

    unsigned int cols = 0;
    result = driver.getCols(cols);
    if (result == true)
        LOG_INFO("getCols: %d", cols);

    // --- Integration time
    unsigned char integration_time = 0;
    result = driver.getIntegrationTime(integration_time);
    if (result == true)
        LOG_INFO("getIntegrationTime: %d", static_cast<int>(integration_time));

    result = driver.setIntegrationTime(5);
    if (result == true)
        LOG_INFO("setIntegrationTime: success");

    // --- Amplitude threshold
    unsigned short amplitude_threshold = 100;
    result = driver.getAmplitudeThreshold(amplitude_threshold);
    if (result == true)
        LOG_INFO("getAmplitudeThreshold: %d", static_cast<int>(amplitude_threshold));

    result = driver.setAmplitudeThreshold(5);
    if (result == true)
        LOG_INFO("setAmplitudeThreshold: success");

    // --- Modulation frequency
    ModulationFrq modulation_frequency = MF_29MHz ;
    result = driver.getModilationFrequency(modulation_frequency);
    if (result == true)
        LOG_INFO("getModilationFrequency: %d", static_cast<int>(modulation_frequency));

    result = driver.setModulationFrequency(MF_29MHz);
    if (result == true)
        LOG_INFO("setModulationFrequency: success");


    // ----------------- TEST: acquisition ---------------
    result = driver.acquire();
    if (result == true)
        LOG_INFO("read: success");

    // --- Distance Image
    base::samples::frame::Frame distance_image;
    result = driver.getDistanceImage(distance_image);
    if(result == true)
        LOG_INFO("getDistanceImage: success");

    // --- Amplitude Image
    base::samples::frame::Frame amplitude_image;
    result = driver.getAmplitudeImage(amplitude_image);
    if(result == true)
        LOG_INFO("getAmplitudeImage: success");

    // --- Confidence Map Image
    base::samples::frame::Frame  confidence_map_image;
    result = driver.getConfidenceMapImage(confidence_map_image);
    if(result == true)
        LOG_INFO("getConfidenceMapImage: success");

    // --- Distance in Cartesian
    base::samples::Pointcloud pointcloud_d;
    result = driver.getDistanceCartesian(pointcloud_d, CP_DOUBLE);
    if (result == true)
        LOG_INFO("getCoordinate: success");

    base::samples::Pointcloud pointcloud_f;
    result = driver.getDistanceCartesian(pointcloud_f, CP_FLOAT);
    if (result == true)
        LOG_INFO("getCoordinate: success");

    base::samples::Pointcloud pointcloud_i;
    result = driver.getDistanceCartesian(pointcloud_i, CP_UINT);
    if (result == true)
        LOG_INFO("getCoordinate: success");

    std::cout << "Distance in Cartesian" << std::endl;
    printPointCloud(pointcloud_d, pointcloud_f, pointcloud_i);

    base::samples::frame::Frame frame_ampl;
    result = driver.getAmplitudeImage(frame_ampl);
    if (result == true)
        LOG_INFO("getAmplitudeImage: success");

    result = driver.close();
    if (result == true)
        LOG_INFO("close: success");


    // open device over USB
    if (driver.openUSB(0) == false) {
        LOG_ERROR("openUSB: error");
        return false;
    }

    // --- Modulation frequency
    modulation_frequency = MF_29MHz ;
    result = driver.getModilationFrequency(modulation_frequency);
    if (result == true) {
        std::string freq_str;
        modulationFreqToStr(modulation_frequency, freq_str);
        LOG_INFO("getModilationFrequency: %s", freq_str.c_str());
    }
    result = driver.close();
    if (result == true)
        LOG_INFO("close: success");

    return 0;
}

void printPointCloud(base::samples::Pointcloud &pointcloud)
{
    if (pointcloud.points.size() > 10)
    {
        for (unsigned int i = 0; i < 10; ++i)
        {
            std::cout << pointcloud.points.at(i).x() << " "
                      << pointcloud.points.at(i).y() << " "
                      << pointcloud.points.at(i).z() << " " << std::endl;
        }
    }
}

void printPointCloud(base::samples::Pointcloud &pointcloud_1,
                     base::samples::Pointcloud &pointcloud_2,
                     base::samples::Pointcloud &pointcloud_3)
{
    if (pointcloud_1.points.size() > 10 && pointcloud_2.points.size() > 10 && pointcloud_3.points.size() > 10)
    {
        for (unsigned int i = 0; i < 10; ++i)
        {
            std::cout << pointcloud_1.points.at(i).x() << ", "
                      << pointcloud_2.points.at(i).x() << ", "
                      << pointcloud_3.points.at(i).x() << "; \t "
                      << pointcloud_1.points.at(i).y() << ", "
                      << pointcloud_2.points.at(i).y() << ", "
                      << pointcloud_3.points.at(i).y() << "; \t "
                      << pointcloud_1.points.at(i).z() << ", "
                      << pointcloud_2.points.at(i).z() << ", "
                      << pointcloud_3.points.at(i).z() << ";" << std::endl;
        }
    }
}

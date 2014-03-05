#include <iostream>
#include <base/logging.h>
#include "SwissRangerDriver.hpp"
#include "SwissRangeDriverLookUp.hpp"
#include <bitset>

using namespace tofcamera_mesa_swissranger;

void printPointCloud(std::vector<base::Vector3d> &point_d,
                     std::vector<Eigen::Matrix<float, 3, 1, Eigen::DontAlign> > &points_f,
                     std::vector<Eigen::Matrix<short, 3, 1, Eigen::DontAlign> > &points_s);

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

    // --- Serial Number
    unsigned int serial_number = 0;
    result = driver.getSerialNumber(serial_number);
    if (result == true)
        LOG_INFO("getSerialNumber: %x", serial_number);

    // --- Firmware Version
    unsigned int firmware_version = 0;
    result = driver.getFirmwareVersion(firmware_version);
    if (result == true)
        LOG_INFO("getFirmwareVersion: %x", firmware_version);

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
    result = driver.turnAcquisitionMode(AM_COR_FIX_PTRN, true);
    //result = driver.turnAcquisitionMode(AM_SW_TRIGGER, false);
    if (result == true)
       LOG_INFO("turnAcquisitionMode: success");

    // --- Timeout
    driver.setTimeout(3000);
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

    result = driver.setIntegrationTime(30);
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
    if (result == true) {
        std::string freq_str;
        modulationFreqToStr(modulation_frequency, freq_str);
        LOG_INFO("getModilationFrequency: %s", freq_str.c_str());
    }

    result = driver.setModulationFrequency(MF_15MHz);
    if (result == true)
        LOG_INFO("setModulationFrequency: success");

    result = driver.setDualIntegrationTime(50);
    if (result == true)
        LOG_INFO("setDualIntegrationTime: success");

    // ----------------- TEST: acquisition ---------------
    result = driver.acquire();
    if (result == true)
        LOG_INFO("read: success");

    // --- Distance Image
    std::vector<uint16_t> distance_image;
    result = driver.getDistanceImage(&distance_image);
    if(result == true)
        LOG_INFO("getDistanceImage: success");

    // --- Amplitude Image
    std::vector<uint16_t> amplitude_image;
    result = driver.getAmplitudeImage(&amplitude_image);
    if(result == true)
        LOG_INFO("getAmplitudeImage: success");

    // --- Confidence Map Image
    std::vector<uint16_t>  confidence_image;
    result = driver.getConfidenceImage(&confidence_image);
    if(result == true)
        LOG_INFO("getConfidenceImage: success");

    // --- Distance in Cartesian
    std::vector<base::Vector3d> points_d;
    result = driver.getPointcloudDouble(points_d);
    if (result == true)
        LOG_INFO("getPointcloudDouble: success");

    std::vector<Eigen::Matrix<float, 3, 1, Eigen::DontAlign> > points_f;
    result = driver.getPointcloudFloat(points_f);
    if (result == true)
        LOG_INFO("getPointcloudFloat: success");

    std::vector<Eigen::Matrix<short, 3, 1, Eigen::DontAlign> > points_s;
    result = driver.getPointcloudShort(points_s);
    if (result == true)
        LOG_INFO("getPointcloudShort: success");

    std::cout << "Distance in Cartesian" << std::endl;
    printPointCloud(points_d, points_f, points_s);

    result = driver.close();
    if (result == true)
        LOG_INFO("close: success");

    return 0;
}

void printPointCloud(std::vector<base::Vector3d> &point_d,
                     std::vector<Eigen::Matrix<float, 3, 1, Eigen::DontAlign> > &points_f,
                     std::vector<Eigen::Matrix<short, 3, 1, Eigen::DontAlign> > &points_s)
{
    if (point_d.size() > 10 && points_f.size() > 10 && points_s.size() > 10)
    {
        for (unsigned int i = 0; i < 10; ++i)
        {
            std::cout << point_d.at(i).x() << ", "
                      << points_f.at(i).x() << ", "
                      << points_s.at(i).x() << "; \t "
                      << point_d.at(i).y() << ", "
                      << points_f.at(i).y() << ", "
                      << points_s.at(i).y() << "; \t "
                      << point_d.at(i).z() << ", "
                      << points_f.at(i).z() << ", "
                      << (unsigned short)points_s.at(i).z() << ";" << std::endl;
        }
    }
}

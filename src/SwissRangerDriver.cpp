#include "SwissRangerDriver.hpp"

using namespace tofcamera_mesa_swissranger;

SwissRangerDriver::SwissRangerDriver()
    : camera_handle_(0), is_open_(false), img_entry_array_(0), img_indexes_(11, -1),
      timeout_(TIMEOUT), image_buffer_(0)
{
    LOG_DEBUG("SwissRangerDriver: constructor");

}

SwissRangerDriver::~SwissRangerDriver()
{
    LOG_DEBUG("SwissRangerDriver: destructor");

    if (isOpen() == true)
        close();
}

bool SwissRangerDriver::openUSB(const unsigned int serial_number)
{
    LOG_DEBUG("SwissRangerDriver: openUSB");

    // close the device to reset all settings to default
    if (isOpen() == true)
    {
        LOG_INFO("SwissRangerDriver: the device is already opened. "
                 "It will be closed and reopened, therefore all "
                 "settings will be set to default.");
        close();
    }

    // open device
    int camera_number = SR_OpenUSB(&camera_handle_, serial_number);
    if (camera_number <= 0)
    {
        LOG_ERROR("SwissRangerDriver: the device could not be opened "
                  "on USB port! Check the USB permission.");
        return false;
    }

    // get image information
    if (init() != true) {
        close();
        return false;
    }

    is_open_ = true;

    LOG_INFO("SwissRangerDriver: the device is opened on USB port.");
    return true;
}

bool SwissRangerDriver::openEthernet(const std::string &ip_address)
{
    LOG_DEBUG("SwissRangerDriver: openEthernet");

    // close the device to reset all settings to default
    if (isOpen() == true)
    {
        LOG_INFO("SwissRangerDriver: the device is already opened. "
                 "It will be closed and reopened, therefore all "
                 "settings will be set to default.");
        close();
    }

    // open device
    int camera_number = SR_OpenETH(&camera_handle_, ip_address.c_str());
    if (camera_number <= 0)
    {
        LOG_ERROR("SwissRangerDriver: the device could not be opened "
                  "on Ethernet port!");
        return false;
    }

    // get image information
    if (init() != true) {
        close();
        return false;
    }

    is_open_ = true;
            
    LOG_INFO("SwissRangerDriver: the device is opened on Ethernet port.");
    return true;
}

bool SwissRangerDriver::init()
{
    // get an array of all avaliable images
    if (initImageList() != true)
        return false;

    // get the size
    if (getCols(cols_) != true)
        return false;

    if (getRows(rows_) != true)
        return false;

    // allocate the buffer with maximal size
    size_t buffer_size = rows_ * cols_ * 3 * sizeof(double);
    image_buffer_ = malloc(buffer_size);
    memset(image_buffer_, 0xaf, buffer_size);

    return true;
}

bool SwissRangerDriver::initImageList()
{
    LOG_DEBUG("SwissRangerDriver: initImageList");

    number_of_images_= SR_GetImageList(camera_handle_, &img_entry_array_);

    // at least 2 images (distance and amplitude) are always available
    // independent of the current acquisition mode
    if (number_of_images_ < 2 || img_entry_array_ == 0)
    {
        LOG_ERROR("SwissRangerDriver: failed to get a list of all available images");
        return false;
    }

    // set the index of images according to the available images in the img_entry_array_
    std::fill (img_indexes_.begin(), img_indexes_.end(), -1);

    for (int i = 0; i < number_of_images_; ++i)
    {
        switch (img_entry_array_[i].imgType)
        {
        case ImgEntry::IT_DISTANCE:
            img_indexes_.at(0) = i;
            break;
        case ImgEntry::IT_AMPLITUDE:
            img_indexes_.at(1) = i;
            break;
        case ImgEntry::IT_INTENSITY:
            img_indexes_.at(2) = i;
            break;
        case ImgEntry::IT_TAP0:
            img_indexes_.at(3) = i;
            break;
        case ImgEntry::IT_TAP1:
            img_indexes_.at(4) = i;
            break;
        case ImgEntry::IT_TAP2:
            img_indexes_.at(5) = i;
            break;
        case ImgEntry::IT_TAP3:
            img_indexes_.at(6) = i;
            break;
        case ImgEntry::IT_SUM_DIFF:
            img_indexes_.at(7) = i;
            break;
        case ImgEntry::IT_CONF_MAP:
            img_indexes_.at(8) = i;
            break;
        case ImgEntry::IT_UNDEFINED:
            img_indexes_.at(9) = i;
            break;
        case ImgEntry::IT_LAST:
            img_indexes_.at(10) = i;
            break;
        default:
            break;

        }
    }

    return true;
}

bool SwissRangerDriver::close()
{
    LOG_DEBUG("SwissRangerDriver: close");

    if (isOpen() == false)
    {
        LOG_INFO("SwissRangerDriver: the device is already closed.");
        return true;
    }

    // close device
    int result = SR_Close(camera_handle_);
    switch(result)
    {
        case 0:        // success
            break;
        case -1:
            LOG_ERROR("SwissRangerDriver: the device could not be closed - "
                      "wrong device");
            return false;
        case -2:
            LOG_ERROR("SwissRangerDriver: the device could not be closed - "
                      "can't release interface.");
            return false;
        case -3:
            LOG_ERROR("SwissRangerDriver: the device could not be closed - "
                      "can't close device.");
            return false;
        default:
            LOG_ERROR("SwissRangerDriver: the device could not be closed - "
                      "unknown error.");
            return false;
    }

    is_open_ = false;
    timeout_ = TIMEOUT;
    camera_handle_ = 0;
    img_entry_array_ = 0;
    number_of_images_ = 0;

    free(image_buffer_);
    image_buffer_ = 0;

    LOG_INFO("SwissRangerDriver: the device is closed.");
    return true;
}

bool SwissRangerDriver::isOpen() const
{
    return is_open_;
}

bool SwissRangerDriver::getLibraryVersion(std::string &version)
{
    LOG_DEBUG("SwissRangerDriver: getLibraryVersion");

    unsigned short sr_version[4];

    int result = SR_GetVersion(sr_version);
    if (result < 0)
    {
        LOG_ERROR("SwissRangerDriver: library version is not avaliable.");
        return false;
    }

    version = boost::lexical_cast<std::string>(sr_version[3]) + "."
            + boost::lexical_cast<std::string>(sr_version[2]) + "."
            + boost::lexical_cast<std::string>(sr_version[1]) + "."
            + boost::lexical_cast<std::string>(sr_version[0]);

    return true;
}

bool SwissRangerDriver::getDeviceInfo(std::string &info)
{
    LOG_DEBUG("SwissRangerDriver: getDeviceInfo");

    char buffer[1024];

    int result = SR_GetDeviceString(camera_handle_, buffer, boost::size(buffer));
    if (result < 0)
    {
        LOG_ERROR("SwissRangerDriver: device information is not avaliable. "
                  "Check if the device is connected and opened.");
        return false;
    }

    info = std::string(buffer);

    return true;
}

bool SwissRangerDriver::setAcquisitionMode(int mode)
{
    LOG_DEBUG("SwissRangerDriver: setAcquisitionMode");

    int result = SR_SetMode(camera_handle_, mode);
    if (result < 0)
    {
        LOG_ERROR("SwissRangerDriver: the acquisition mode could not be set. "
                  "The supported acquistion mode depend on the camera type (SR3/SR4).");
        return false;
    }

    // print the information of acquisition mode
    std::string modes_str;
    acquireModesToStr(mode, modes_str);
    LOG_INFO("SwissRangerDriver: the acquisition mode was set to %s", modes_str.c_str());

    // print the warning if the acquisition mode is only for internal testing
    warnInternalAcquisitionMode(mode);

    // update the image buffer pointers:
    // some changes in acqusition mode can result a reallocation of the image buffers,
    // therefore the stored pointers (in img_entry_array_) will become invalid.
    // (s. sensor documentation)
    if (initImageList() != true)
        return false;

    return true;
}

void SwissRangerDriver::warnInternalAcquisitionMode(int mode)
{
    if ((mode & AM_TOGGLE_FRQ) == AM_TOGGLE_FRQ
            || (mode & AM_SW_ANF) == AM_SW_ANF
            || (mode & AM_RESERVED0) == AM_RESERVED0
            || (mode & AM_RESERVED1) == AM_RESERVED1
            || (mode & AM_MEDIANCROSS) == AM_MEDIANCROSS)
        LOG_WARN("SwissRangerDriver: according to the device API some acquisition "
                 "modes are just for internal testing and should not be used.");
}

bool SwissRangerDriver::getAcquisitionMode(int &mode)
{
    LOG_DEBUG("SwissRangerDriver: getAcquisitionMode");

    int result = SR_GetMode(camera_handle_);

    if (result <= 0)
    {
        LOG_ERROR("SwissRangerDriver: could not get the acquisition mode. "
                  "Check if the device is connected and opened properly.");
        return false;
    }

    mode = result;

    return true;
}

bool SwissRangerDriver::turnAcquisitionMode(int mode, bool turnOn)
{
    LOG_DEBUG("SwissRangerDriver: turnAcquisitionMode");

    int current_mode = 0;
    bool result = getAcquisitionMode(current_mode);

    if (result == false)
    {
        LOG_ERROR("SwissRangerDriver: failed to turn acquisition mode.");
        return false;
    }

    if (turnOn == true)
        mode = current_mode | mode;
    else
        mode = current_mode & (~mode);

    result = setAcquisitionMode(mode);

    if (result == false)
    {
        LOG_ERROR("SwissRangerDriver: failed to turn acquisition mode.");
        return false;
    }

    return true;
}

void SwissRangerDriver::setTimeout(int ms)
{
    LOG_DEBUG("SwissRangerDriver: setTimeout");

    SR_SetTimeout(camera_handle_, ms);
    timeout_ = ms;
}

int SwissRangerDriver::getTimeout()
{
    LOG_DEBUG("SwissRangerDriver: getTimeout");

    return timeout_;
}

bool SwissRangerDriver::getRows(unsigned int &rows)
{
    LOG_DEBUG("SwissRangerDriver: getRows");

    unsigned int result = SR_GetRows(camera_handle_);
    if (result == 0)
    {
        LOG_ERROR("SwissRangerDriver: could not get the number of rows. "
                  "Check if the device is connected and opened properly.");
        return false;
    }

    rows = result;

    return true;
}

bool SwissRangerDriver::getCols(unsigned int &cols)
{
    LOG_DEBUG("SwissRangerDriver: getCols");

    unsigned int result = SR_GetCols(camera_handle_);
    if (result == 0)
    {
        LOG_ERROR("SwissRangerDriver: could not get the number of colums.");
        return false;
    }

    cols = result;

    return true;
}

bool SwissRangerDriver::setIntegrationTime(unsigned char time)
{
    LOG_DEBUG("SwissRangerDriver: setIntegrationTime");

    int result = SR_SetIntegrationTime(camera_handle_, time);

    // result value indicate the number of bytes transferred to or from the camera during an operation
    if (result < 2)
    {
        LOG_ERROR("SwissRangerDriver: could not set the integration time.");
        return false;
    }

    return true;
}

bool SwissRangerDriver::getIntegrationTime(unsigned char &time)
{
    LOG_DEBUG("SwissRangerDriver: getIntegrationTime");

    unsigned char result = SR_GetIntegrationTime(camera_handle_);
    if (static_cast<int>(result) <= 0)
    {
        LOG_ERROR("SwissRangerDriver: could not get the integration time.");
        return false;
    }

    time = result;

    return true;
}

bool SwissRangerDriver::setAmplitudeThreshold(unsigned short threshold)
{
    LOG_DEBUG("SwissRangerDriver: setAmplitudeThreshold");

    int result = SR_SetAmplitudeThreshold(camera_handle_, threshold);

    // result value indicate the number of bytes transferred to or from the camera during an operation
    if (result < 4)
    {
        LOG_ERROR("SwissRangerDriver: could not set the amplitude threshold.");
        return false;
    }

    return true;
}

bool SwissRangerDriver::getAmplitudeThreshold(unsigned short &threshold)
{
    LOG_DEBUG("SwissRangerDriver: getAmplitudeThreshold");

    unsigned short result = SR_GetAmplitudeThreshold(camera_handle_);
    if (result < 0)
    {
        LOG_ERROR("SwissRangerDriver: could not get the amplitude threshold.");
        return false;
    }

    threshold = result;

    return true;
}

bool SwissRangerDriver::setModulationFrequency(ModulationFrq frequency)
{
    LOG_DEBUG("SwissRangerDriver: setModulationFrequency");

    int result = SR_SetModulationFrequency(camera_handle_, frequency);
    if (result < 0)
    {
        LOG_ERROR("SwissRangerDriver: could not set the modulation frequency. "
                  "The supported frequencies depend on the camera type (SR3/SR4).");
        return false;
    }

    // print the information of modulation frequency
    std::string freq_str;
    modulationFreqToStr(frequency, freq_str);
    LOG_INFO("SwissRangerDriver: the modulation frequency was set to %s", freq_str.c_str());

    // print the warning if the modulation frequency is only for internal testing
    warnInternalModulationFrequency(frequency);

    return true;
}

void SwissRangerDriver::warnInternalModulationFrequency(ModulationFrq frequency)
{
    if (frequency == MF_60MHz
            || frequency == MF_10MHz)
        LOG_WARN("SwissRangerDriver: according to the device API some modulation frequency "
                 "are just for internal testing and should not be used.");
}

bool SwissRangerDriver::getModilationFrequency(ModulationFrq &frequency)
{
    LOG_DEBUG("SwissRangerDriver: getModilationFrequency");

    ModulationFrq result = SR_GetModulationFrequency(camera_handle_);
    if (static_cast<int>(result) < 0)
    {
        LOG_ERROR("SwissRangerDriver: could not get the modulation frequency.");
        return false;
    }

    frequency = result;

    return true;
}

bool SwissRangerDriver::setAutoExposure(unsigned char min_int_time, unsigned char max_int_time,
                                        unsigned char percent_over_pos, unsigned char desired_pos)
{
    LOG_DEBUG("SwissRangerDriver: setAutoExposure");

    int result = SR_SetAutoExposure(camera_handle_, min_int_time, max_int_time, percent_over_pos, desired_pos);
    if (result < 0)
    {
        LOG_ERROR("SwissRangerDriver: could not set the auto exposure.");
        return false;
    }

    return true;
}


bool SwissRangerDriver::acquire()
{
    LOG_DEBUG("SwissRangerDriver: acquire");

    int transfered_bytes = SR_Acquire(camera_handle_); //acquire image
    if (transfered_bytes < 0)
    {
        LOG_ERROR("SwissRangerDriver: the acquisition failed.");
        return false;
    }

    return true;
}

bool SwissRangerDriver::get3DCoordinates(base::samples::Pointcloud &pointcloud, const TMS_CoordPercision persicion)
{
    LOG_DEBUG("SwissRangerDriver: get3DCoordinates");

    if (image_buffer_ == 0)
    {
        LOG_ERROR("SwissRangerDriver: failed to get image.");
        return false;
    }

    switch (persicion)
    {
    case CP_DOUBLE:
        {
            int pitch = 3 * sizeof(double);
            double* buf = (double*) image_buffer_;

            // transfrom the distance image from spherical coordinates to cartesian coordinates
            // coordinate in meter
            int result = SR_CoordTrfDbl(camera_handle_,
                    &buf[0], &buf[1], &buf[2],
                    pitch, pitch, pitch);

            if (result < 0)
            {
                LOG_ERROR("SwissRangerDriver: could not transform the distance image to cartesian coordinate with double percistion.");
                return false;
            }

            for (unsigned int i = 0; i < (rows_ * cols_ * 3); i += 3)
            {
                pointcloud.points.push_back(base::Point(buf[i], buf[i+1], buf[i+2]));
            }
        }

       break;
   case CP_FLOAT:
        {
            int pitch = 3 * sizeof(float);
            float* buf = (float*) image_buffer_;

            // transfrom the distance image from spherical coordinates to cartesian coordinates
            // coordinate in meter
            int result = SR_CoordTrfFlt(camera_handle_,
                    &buf[0], &buf[1], &buf[2],
                    pitch, pitch, pitch);

            if (result < 0)
            {
                LOG_ERROR("SwissRangerDriver: could not transform the distance image to cartesian coordiante with float percistion.");
                return false;
            }

            for (unsigned int i = 0; i < (rows_ * cols_ * 3); i += 3)
            {
                pointcloud.points.push_back(base::Point(buf[i], buf[i+1], buf[i+2]));
            }
        }
        break;
   case CP_UINT:
        {
            int pitch = 2 * sizeof(short) + sizeof(unsigned short);
            short* buf = (short*) image_buffer_;

            // transfrom the distance image from spherical coordinates to cartesian coordinates
            // coordinate in mm
            int result = SR_CoordTrfUint16(camera_handle_,
                                           &buf[0], &buf[1], (unsigned short*) &buf[2],
                                           pitch, pitch, pitch);

            if (result < 0)
            {
                LOG_ERROR("SwissRangerDriver: could not transform the distance image to cartesian coordiante with uint percistion.");
                return false;
            }

            for (unsigned int i = 0; i < (rows_ * cols_ * 3); i += 3)
            {
                pointcloud.points.push_back(base::Point(buf[i], buf[i+1], (unsigned short)buf[i+2]));
            }
        }
        break;
   default:
        LOG_ERROR("SwissRangerDriver: unknown percision");
        break;
   }

   pointcloud.time = base::Time::now();

   return true;
}

bool SwissRangerDriver::getDistanceImage(base::samples::frame::Frame &frame)
{
    LOG_DEBUG("SwissRangerDriver: getDistanceImage");

    if (img_entry_array_ == 0 || img_indexes_.at(ImgEntry::IT_DISTANCE) == -1)
    {
        LOG_ERROR("SwissRangerDriver: the distance image is not available");
        return false;
    }

    bool result = getImage(frame, img_indexes_.at(ImgEntry::IT_DISTANCE));
    if (result == false)
    {
        LOG_ERROR("SwissRangerDriver:: the distance image can not be accessed");
        return false;
    }

    return true;
}

bool SwissRangerDriver::getAmplitudeImage(base::samples::frame::Frame &frame)
{
    LOG_DEBUG("SwissRangerDriver: getAmplitudeImage");

    if (img_entry_array_ == 0 || img_indexes_.at(ImgEntry::IT_AMPLITUDE) == -1)
    {
        LOG_ERROR("SwissRangerDriver: the amplitude image is not available");
        return false;
    }

    bool result = getImage(frame, img_indexes_.at(ImgEntry::IT_AMPLITUDE));

    if (result == false)
    {
        LOG_ERROR("SwissRangerDriver:: the amplitude image can not be accessed");
        return false;
    }

    return true;
}

bool SwissRangerDriver::getConfidenceImage(base::samples::frame::Frame &frame)
{
    LOG_DEBUG("SwissRangerDriver: getConfidenceImage");

    if (img_entry_array_ == 0 || img_indexes_.at(ImgEntry::IT_CONF_MAP) == -1)
    {
        LOG_ERROR("SwissRangerDriver: the confidence image is not available");
        return false;
    }

    bool result = getImage(frame, img_indexes_.at(ImgEntry::IT_CONF_MAP));
    if (result == false)
    {
        LOG_ERROR("SwissRangerDriver:: the confidence image can not be accessed");
        return false;
    }

    return true;
}

bool SwissRangerDriver::isConfidenceImageAvailable()
{
    if (img_indexes_.at(ImgEntry::IT_CONF_MAP) == -1)
        return false;
    else
        return true;
}

bool SwissRangerDriver::getImage(base::samples::frame::Frame &frame, const int index)
{
    unsigned short* data = (unsigned short*)img_entry_array_[index].data;
    if (data == 0)
        return false;

    unsigned int width = (unsigned int)img_entry_array_[index].width;
    unsigned int height = (unsigned int)img_entry_array_[index].height;

    frame.init(width, height, 16);
    frame.setImage((char*) data, (width * height) * 2);

    frame.time = base::Time::now();

    return true;
}

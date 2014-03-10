#include "SwissRangerDriver.hpp"

using namespace tofcamera_mesasr;

SwissRangerDriver::SwissRangerDriver()
    : camera_handle_(0), is_open_(false), img_entry_array_(0), img_indexes_(ImgEntry::IT_LAST + 1, -1),
      timeout_(TIMEOUT), confidence_threshold_(0), remove_zero_points_(false), pointcloud_buffer_(0)
{
    LOG_DEBUG("SwissRangerDriver: constructor");
}

SwissRangerDriver::~SwissRangerDriver()
{
    LOG_DEBUG("SwissRangerDriver: destructor");

    //if (isOpen() == true)
    //    close();
}

bool SwissRangerDriver::openUSB(const unsigned int serial_number)
{
    LOG_DEBUG("SwissRangerDriver: openUSB");

    // close the device to reset all settings to default
    if (isOpen() == true)
    {
        LOG_WARN("SwissRangerDriver: the device is already opened. "
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
        LOG_WARN("SwissRangerDriver: the device is already opened. "
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
    LOG_DEBUG("SwissRangerDriver: init");

    // get an array of all avaliable images
    if (initImageList() != true)
        return false;

    // get the size
    if (getCols(cols_) != true)
        return false;

    if (getRows(rows_) != true)
        return false;

    // allocate the buffer with maximal size to store a pointcloud 
    size_t buffer_size = rows_ * cols_ * 3 * sizeof(double);
    pointcloud_buffer_ = malloc(buffer_size);
    memset(pointcloud_buffer_, 0xaf, buffer_size);

    return true;
}

bool SwissRangerDriver::initImageList()
{
    LOG_DEBUG("SwissRangerDriver: initImageList");

    num_of_images_= SR_GetImageList(camera_handle_, &img_entry_array_);

    // at least 2 images (distance and amplitude) are always available
    // independent of the current acquisition mode
    if (num_of_images_ < 2 || img_entry_array_ == 0)
    {
        LOG_ERROR("SwissRangerDriver: failed to get a list of all available images");
        return false;
    }

    // set the index of images according to the available images in the img_entry_array_
    std::fill(img_indexes_.begin(), img_indexes_.end(), -1);

    for (int i = 0; i < num_of_images_; ++i)
    {
        switch (img_entry_array_[i].imgType)
        {
        case ImgEntry::IT_DISTANCE:
            img_indexes_.at(ImgEntry::IT_DISTANCE) = i;
            break;
        case ImgEntry::IT_AMPLITUDE:
            img_indexes_.at(ImgEntry::IT_AMPLITUDE) = i;
            break;
        case ImgEntry::IT_INTENSITY:
            img_indexes_.at(ImgEntry::IT_INTENSITY) = i;
            break;
        case ImgEntry::IT_TAP0:
            img_indexes_.at(ImgEntry::IT_TAP0) = i;
            break;
        case ImgEntry::IT_TAP1:
            img_indexes_.at(ImgEntry::IT_TAP1) = i;
            break;
        case ImgEntry::IT_TAP2:
            img_indexes_.at(ImgEntry::IT_TAP2) = i;
            break;
        case ImgEntry::IT_TAP3:
            img_indexes_.at(ImgEntry::IT_TAP3) = i;
            break;
        case ImgEntry::IT_SUM_DIFF:
            img_indexes_.at(ImgEntry::IT_SUM_DIFF) = i;
            break;
        case ImgEntry::IT_CONF_MAP:
            img_indexes_.at(ImgEntry::IT_CONF_MAP) = i;
            break;
        case ImgEntry::IT_UNDEFINED:
            img_indexes_.at(ImgEntry::IT_UNDEFINED) = i;
            break;
        case ImgEntry::IT_LAST:
            img_indexes_.at(ImgEntry::IT_LAST) = i;
            break;
        default:
            break;
        }
    }

    LOG_INFO("SwissRangerDriver: the list of avaliable images is initialized.");

    return true;
}

bool SwissRangerDriver::close()
{
    LOG_DEBUG("SwissRangerDriver: close");

    if (isOpen() == false)
    {
        LOG_WARN("SwissRangerDriver: the device is already closed.");
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
    num_of_images_ = 0;

    free(pointcloud_buffer_);
    pointcloud_buffer_ = 0;

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

bool SwissRangerDriver::getSerialNumber(unsigned int &number)
{
    LOG_DEBUG("SwissRangerDriver: getSerialNumber");

    unsigned int result = SR_ReadSerial(camera_handle_);

    std::cout << "SERIAL NUMBER " << result << std::endl;

    if (result == 0xffffffff)
    {
        LOG_ERROR("SwissRangerDriver: serial number is not avaliable. ");
        return false;
    }

    number = result;

    return true;
}

bool SwissRangerDriver::getFirmwareVersion(unsigned int &version)
{
    LOG_DEBUG("SwissRangerDriver: getFirmwareVersion");

    unsigned char result = SR_GetReg(camera_handle_, 46);

    if (result == 0xFF)
    {
        LOG_ERROR("SwissRangerDriver: firmware information is not avaliable. ");
        return false;        
    }

    version = (unsigned int) result;

    return true;
}

bool SwissRangerDriver::setAcquisitionMode(int mode)
{
    LOG_DEBUG("SwissRangerDriver: setAcquisitionMode");

    int result = SR_SetMode(camera_handle_, mode);
    if (result < 0)
    {
        LOG_ERROR("SwissRangerDriver: the acquisition mode could not be set. "
                  "The supported acquisition mode depends on the camera type (SR3K/SR4K).");
        return false;
    }

    // print the information of acquisition mode
    std::string modes_str;
    acquireModesToStr(mode, modes_str);

    // print the warning if the acquisition mode is only for internal testing
    warnInternalAcquisitionMode(mode);

    LOG_INFO("SwissRangerDriver: the acquisition mode was set to %s", modes_str.c_str());

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

    LOG_INFO("SwissRangerDriver: the timeout was set to %d", timeout_);
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

    // result value indi1cate the number of bytes transferred to 
    // or from the camera during an operation
    if (result < 2)
    {
        LOG_ERROR("SwissRangerDriver: could not set the integration time. " 
            "Check the time value. It should be from 0 to 255.");
        return false;
    }

    LOG_INFO("SwissRangerDriver: the integration time was set to %d", (int)time);

    return true;
}

bool SwissRangerDriver::getIntegrationTime(unsigned char &time)
{
    LOG_DEBUG("SwissRangerDriver: getIntegrationTime");

    if (isOpen())
    {
        time = SR_GetIntegrationTime(camera_handle_);
        return true;
    }
    else 
    {
        LOG_ERROR("SwissRangerDriver: could not get the integration time. "
            "The camera is not opened.");
        return false;
    }
}

bool SwissRangerDriver::setDualIntegrationTime(int ratio)
{
    LOG_DEBUG("SwissRangerDriver: setDualIntegrationTime");

    // check firmware
    unsigned char fw = SR_GetReg(camera_handle_,46);
    if (fw < 0x73)
    {
        std::stringstream stream;
        stream << std::hex << (int)fw;
        LOG_ERROR_S << "SwissRangerDriver: could not set the dual integration time. "
                    << "DualIntegrationTime is available for the firmware >= 0x73, "
                    << "the current firmware is 0x"
                    << stream.str() << ". "
                    << "Therefore, the camera is working in single integration mode."
                    << std::endl;
        return false;
    }

    int result = SR_SetDualIntegrationTime(camera_handle_, ratio);

    if (result < 0)
    {
        LOG_ERROR("SwissRangerDriver: could not set the dual integration time. "
            "Check the ratio value. It should be from 0 to 100.");
        return false;
    }

    LOG_INFO("SwissRangerDriver: the dual integration time was set to %d", (int)ratio);

    return true;
}

bool SwissRangerDriver::setAmplitudeThreshold(unsigned short threshold)
{
    LOG_DEBUG("SwissRangerDriver: setAmplitudeThreshold");

    int result = SR_SetAmplitudeThreshold(camera_handle_, threshold);

    // result value indicate the number of bytes transferred 
    // to or from the camera during an operation
    if (result < 4)
    {
        LOG_ERROR("SwissRangerDriver: could not set the amplitude threshold.");
        return false;
    }

    LOG_INFO("SwissRangerDriver: the amplitude threshold was set to %d", (int)threshold);

    return true;
}

bool SwissRangerDriver::getAmplitudeThreshold(unsigned short &threshold)
{
    LOG_DEBUG("SwissRangerDriver: getAmplitudeThreshold");

    if (isOpen())
    {
        threshold = SR_GetAmplitudeThreshold(camera_handle_);
        return true;
    } 
    else
    {
        LOG_ERROR("SwissRangerDriver: could not get the amplitude threshold. "
            "The camera is not opened.");
        return false;        
    }
}

void SwissRangerDriver::setConfidenceThreshold(unsigned short threshold)
{
    LOG_DEBUG("SwissRangerDriver: setConfidenceThreshold");
    confidence_threshold_ = threshold;
}

unsigned short SwissRangerDriver::getConfidenceThreshold()
{
    return confidence_threshold_;
}

bool SwissRangerDriver::setModulationFrequency(ModulationFrq frequency)
{
    LOG_DEBUG("SwissRangerDriver: setModulationFrequency");

    int result = SR_SetModulationFrequency(camera_handle_, frequency);

    // TODO: returns always 0, even if the camera is not connected
    if (result < 0)
    {
        LOG_ERROR("SwissRangerDriver: could not set the modulation frequency. "
                  "The supported frequencies depend on the camera type (SR3K/SR4K).");
        return false;
    } else if (result == 1)
    {
        LOG_WARN("SwissRangerDriver: there is no calibration data for this frequency.");
    }

    // print the information of modulation frequency
    std::string freq_str;
    modulationFreqToStr(frequency, freq_str);

    // print the warning if the modulation frequency is only for internal testing
    warnInternalModulationFrequency(frequency);

    LOG_INFO("SwissRangerDriver: the modulation frequency was set to %s", freq_str.c_str());

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

    if (result >= MF_LAST)
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

    LOG_INFO("SwissRangerDriver: the auto exposure values were set to %d, %d, %d, %d", (int)min_int_time, (int)max_int_time, (int)percent_over_pos, (int)desired_pos);

    return true;
}

bool SwissRangerDriver::isConfidenceImageAvailable()
{
    if (img_entry_array_ == 0 || img_indexes_.at(ImgEntry::IT_CONF_MAP) == -1)
        return false;
    else
        return true;
}

bool SwissRangerDriver::acquire()
{
    LOG_DEBUG("SwissRangerDriver: acquire");

    int transfered_bytes = SR_Acquire(camera_handle_); //acquire image

    // TODO: check the number of transfered bytes of all images
    // current state: returned just the number of bytes of two images
    // even if the confidence map is turned on 
    if (transfered_bytes < 0)
    {
        LOG_ERROR("SwissRangerDriver: the acquisition failed.");
        return false;
    }

    return true;
}

bool SwissRangerDriver::getDistanceImage(std::vector<uint16_t> *image)
{
    LOG_DEBUG("SwissRangerDriver: getDistanceImage");

    if (img_entry_array_ == 0 || img_indexes_.at(ImgEntry::IT_DISTANCE) == -1)
    {
        LOG_ERROR("SwissRangerDriver: the distance image is not available");
        return false;
    }

    // get distance image data from device
    if (getImage(img_indexes_.at(ImgEntry::IT_DISTANCE), image) == false)
        return false;

    // the distance data contain reserved pixels
    // remove them from the frame
    removeReservedBits(image);

    return true;
}

bool SwissRangerDriver::getAmplitudeImage(std::vector<uint16_t> *image)
{
    LOG_DEBUG("SwissRangerDriver: getAmplitudeImage");

    if (img_entry_array_ == 0 || img_indexes_.at(ImgEntry::IT_AMPLITUDE) == -1)
    {
        LOG_ERROR("SwissRangerDriver: the amplitude image is not available");
        return false;
    }

    // get amplitude image data from device
    if (getImage(img_indexes_.at(ImgEntry::IT_AMPLITUDE), image) == false)
        return false;

    // the amplitude data contain reserved pixels
    // remove them from the frame
    removeReservedBits(image);

    return true;
}

bool SwissRangerDriver::getConfidenceImage(std::vector<uint16_t> *image)
{
    LOG_DEBUG("SwissRangerDriver: getConfidenceImage");

    if (isConfidenceImageAvailable() == false)
        return false;

    // get confidence image data from device
    if (getImage(img_indexes_.at(ImgEntry::IT_CONF_MAP), image) == false)
        return false;

    return true;
}

bool SwissRangerDriver::getImage(int image_index, std::vector<uint16_t> *buffer)
{
    LOG_DEBUG("SwissRangerDriver: getImage");

    // get the pointer to the image buffer of required image type
    uint16_t *data_ptr = (unsigned short*) img_entry_array_[image_index].data;
    if (data_ptr == 0)
    {
        LOG_ERROR("Swissranger: can not access the image");
        return false;
    }

    // the size of image buffer
    uint32_t data_length = (uint32_t)img_entry_array_[image_index].width
                            * (uint32_t)img_entry_array_[image_index].height;

    // copy the image data into buffer
    *buffer = std::vector<uint16_t>(data_ptr, data_ptr + data_length);

    return true;
}

void SwissRangerDriver::removeReservedBits(std::vector<uint16_t> *buffer)
{
    // remove last two least significants bits
    for (unsigned int i = 0; i < buffer->size(); ++i)
    {
        buffer->at(i) = buffer->at(i) >> 2;
    }
}

void SwissRangerDriver::turnRemoveZeroPoints(bool turn_on)
{
    remove_zero_points_ = turn_on;
}

bool SwissRangerDriver::isRemoveZeroPoints()
{
    return remove_zero_points_;
}

bool SwissRangerDriver::getPointcloudDouble(std::vector<base::Vector3d> &points)
{
    LOG_DEBUG("SwissRangerDriver: getPointcloudDouble");

    points.clear();

    if (pointcloud_buffer_ == 0)
    {
        LOG_ERROR("SwissRangerDriver: failed to get pointcloud double.");
        return false;
    }

    int pitch = 3 * sizeof(double);
    double* buf = (double*) pointcloud_buffer_;

    // transfrom the distance image from spherical coordinates to cartesian coordinates
    // coordinate in meter
    int result = SR_CoordTrfDbl(camera_handle_,
            &buf[0], &buf[1], &buf[2],
            pitch, pitch, pitch);

    if (result < 0)
    {
        LOG_ERROR("SwissRangerDriver: could not transform the distance image to "
            "cartesian coordinate with double precision.");
        return false;
    }

    int num_of_points = rows_ * cols_;
    int buffer_size = num_of_points * 3;

    points.resize(num_of_points);
    std::copy(buf, buf + buffer_size, points[0].data());

    // filter out by confidence value
    if (confidence_threshold_ > 0)
        thresholdByConfidence<base::Vector3d>(points);

    // remove all points with zero coordinates
    if (isRemoveZeroPoints() == true)
        removeZeroPoints<base::Vector3d>(points);

    return true;
}

bool SwissRangerDriver::getPointcloudFloat(std::vector<Eigen::Matrix<float, 3, 1, Eigen::DontAlign> > &points)
{
    LOG_DEBUG("SwissRangerDriver: getPointcloudFloat");

    points.clear();

    if (pointcloud_buffer_ == 0)
    {
        LOG_ERROR("SwissRangerDriver: failed to get pointcloud float.");
        return false;
    }

    int pitch = 3 * sizeof(float);
    float* buf = (float*) pointcloud_buffer_;

    // transfrom the distance image from spherical coordinates to cartesian coordinates
    // coordinate in meter
    int result = SR_CoordTrfFlt(camera_handle_,
            &buf[0], &buf[1], &buf[2],
            pitch, pitch, pitch);

    if (result < 0)
    {
        LOG_ERROR("SwissRangerDriver: could not transform the distance image to "
            "cartesian coordiante with float precision.");
        return false;
    }

    int num_of_points = rows_ * cols_;
    int buffer_size = num_of_points * 3;

    points.resize(num_of_points);
    std::copy(buf, buf + buffer_size, points[0].data());    

    // filter out by confidence value
    if (confidence_threshold_ > 0)
        thresholdByConfidence<Eigen::Matrix<float, 3, 1, Eigen::DontAlign> >(points);  

    // remove all points with zero coordinates
    if (isRemoveZeroPoints() == true)
        removeZeroPoints<Eigen::Matrix<float, 3, 1, Eigen::DontAlign> >(points);          

    return true;
}

bool SwissRangerDriver::getPointcloudShort(std::vector<Eigen::Matrix<short, 3, 1, Eigen::DontAlign> > &points)
{
    LOG_DEBUG("SwissRangerDriver: getPointcloudShort");

    points.clear();

    if (pointcloud_buffer_ == 0)
    {
        LOG_ERROR("SwissRangerDriver: failed to get pointcloud short.");
        return false;
    }

    int pitch = 2 * sizeof(short) + sizeof(unsigned short);
    short* buf = (short*) pointcloud_buffer_;

    // transfrom the distance image from spherical coordinates to cartesian coordinates
    // coordinate in mm
    // note: x, y are short and z is ushort
    int result = SR_CoordTrfUint16(camera_handle_,
                                   &buf[0], &buf[1], (unsigned short*) &buf[2],
                                   pitch, pitch, pitch);

    if (result < 0)
    {
        LOG_ERROR("SwissRangerDriver: could not transform the distance image to "
            "cartesian coordiante with short precision.");
        return false;
    }

    int num_of_points = rows_ * cols_;
    int buffer_size = num_of_points * 3;

    points.resize(num_of_points);
    std::copy(buf, buf + buffer_size, points[0].data());  

    // filter out by confidence value
    if (confidence_threshold_ > 0)
        thresholdByConfidence<Eigen::Matrix<short, 3, 1, Eigen::DontAlign> >(points);    

    // remove all points with zero coordinates
    if (isRemoveZeroPoints() == true)
        removeZeroPoints<Eigen::Matrix<short, 3, 1, Eigen::DontAlign> >(points);     

    return true;
}

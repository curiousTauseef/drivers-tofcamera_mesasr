#ifndef _SWISSRANGERDRIVERLOOKUP_HPP_
#define _SWISSRANGERDRIVERLOOKUP_HPP_

#include "libMesaSR.h"

namespace tofcamera_mesasr
{

    inline static void acquireModeToStrList(int mode, std::vector<std::string> &mode_str_list)
    {
        mode_str_list.clear();
        if ((mode & AM_COR_FIX_PTRN) == AM_COR_FIX_PTRN)
            mode_str_list.push_back("noise correction pattern");
        if ((mode & AM_MEDIAN) == AM_MEDIAN)
            mode_str_list.push_back("3x3 Median filter");
        if ((mode & AM_TOGGLE_FRQ) == AM_TOGGLE_FRQ)
            mode_str_list.push_back("toggle frame from 19 to 21 MHz");
        if ((mode & AM_CONV_GRAY) == AM_CONV_GRAY)
            mode_str_list.push_back("convert gray mode");
        if ((mode & AM_SW_ANF) == AM_SW_ANF)
            mode_str_list.push_back("7x7 software adaptive neighborhood filter");
        if ((mode & AM_RESERVED0) == AM_RESERVED0)
            mode_str_list.push_back("2 tap difference mode");
        if ((mode & AM_RESERVED1) == AM_RESERVED1)
            mode_str_list.push_back("more precise coordinate transformations for small distances");
        if ((mode & AM_CONF_MAP) == AM_CONF_MAP)
            mode_str_list.push_back("a confidence map processing");
        if ((mode & AM_HW_TRIGGER) == AM_HW_TRIGGER)
            mode_str_list.push_back("hardware trigger");
        if ((mode & AM_SW_TRIGGER) == AM_SW_TRIGGER)
            mode_str_list.push_back("software trigger");
        if ((mode & AM_DENOISE_ANF) == AM_DENOISE_ANF)
            mode_str_list.push_back("3x3 hardware adaptive neighborhood filter");
        if ((mode & AM_MEDIANCROSS) == AM_MEDIANCROSS)
            mode_str_list.push_back("3x3 cross-median filter");
        if ((mode & AM_NO_AMB) == AM_NO_AMB)
            mode_str_list.push_back("no ambiguity mode");
    }

    inline static void acquireModesToStr(int mode, std::string &mode_str)
    {
        if (!mode_str.empty())
            mode_str.clear();

        // convert mode to string and get a list of mode strings
        std::vector<std::string> mode_str_list;
        acquireModeToStrList(mode, mode_str_list);

        // generate a string from list
        std::vector<std::string>::iterator it_end = mode_str_list.end();
        for (std::vector<std::string>::iterator it = mode_str_list.begin() ; it != it_end; ++it)
        {
            mode_str.append(*it);
            if (it != (it_end - 1) )
                mode_str.append(", ");
        }
    }

    inline static void modulationFreqToStr(ModulationFrq frequency, std::string &freq_str)
    {
        switch (frequency)
        {
            case MF_40MHz:
                freq_str.assign("MF_40Hz");
                break;
            case MF_30MHz:
                freq_str.assign("MF_30MHz");
                break;
            case MF_21MHz:
                freq_str.assign("MF_21MHz");
                break;
            case MF_20MHz:
                freq_str.assign("MF_20MHz");
                break;
            case MF_19MHz:
                freq_str.assign("MF_19MHz");
                break;
            case MF_60MHz:
                freq_str.assign("MF_60MHz");
                break;
            case MF_15MHz:
                freq_str.assign("MF_15MHz");
                break;
            case MF_10MHz:
                freq_str.assign("MF_10MHz");
                break;
            case MF_29MHz:
                freq_str.assign("MF_29MHz");
                break;
            case MF_31MHz:
                freq_str.assign("MF_31MHz");
                break;
            case MF_14_5MHz:
                freq_str.assign("MF_14_5MHz");
                break;
            case MF_15_5MHz:
                freq_str.assign("MF_15_5MHz");
                break;
            default:
                freq_str.assign("Unknown frequency");
                break;
        }
    }

}

#endif // _SWISSRANGERDRIVERLOOKUP_HPP_

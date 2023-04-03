#pragma once
#include "BHI260APSensor.hpp"
using namespace std;

namespace Motion
{
    esp_err_t initBHy2(BHI260APSensor *motionSensor, bhy2_fifo_parse_callback_t callback);
}
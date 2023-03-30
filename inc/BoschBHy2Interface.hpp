#pragma once
#include "BHI260ABSensor.hpp"
using namespace std;

namespace Motion
{
    esp_err_t initBHy2(BHI260ABSensor *motionSensor);
}
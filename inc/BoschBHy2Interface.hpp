#pragma once
#include "BHI260ABSensor.hpp"
using namespace std;

esp_err_t initBHy2(Motion::BHI260ABSensor *motionSensor);
void startBHy2Loop();

#include "BHI260ABSensor.hpp"
#include "BoschBHy2Interface.hpp"

namespace Motion
{
    esp_err_t BHI260ABSensor::init()
    {
        return initBHy2(this);
    }

    void BHI260ABSensor::startSensorLoop()
    {
        startBHy2Loop();
    }
}

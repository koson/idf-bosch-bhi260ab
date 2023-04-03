#include "BHI260APSensor.hpp"
#include "BoschBHy2Interface.hpp"

namespace Motion
{
    esp_err_t BHI260APSensor::init()
    {
        return initBHy2(this);
    }
}

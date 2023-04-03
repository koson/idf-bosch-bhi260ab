#include "BHI260APSensor.hpp"
#include "BoschBHy2Interface.hpp"

namespace Motion
{
    esp_err_t BHI260APSensor::init(bhy2_fifo_parse_callback_t callback)
    {
        return initBHy2(this, callback);
    }
}

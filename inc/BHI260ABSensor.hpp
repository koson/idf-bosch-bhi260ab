#pragma once
#include "I2C.hpp"

using namespace std;
using namespace Components;

namespace Motion
{
    class BHI260ABSensor
    {
    private:
        shared_ptr<I2CMaster> i2cMasterBus;

    public:
        BHI260ABSensor(shared_ptr<I2CMaster> i2cMasterBus)
        {
            this->i2cMasterBus = i2cMasterBus;
        }
        shared_ptr<I2CMaster> getBus()
        {
            return this->i2cMasterBus;
        }
        esp_err_t init();
        void startSensorLoop();
    };
}

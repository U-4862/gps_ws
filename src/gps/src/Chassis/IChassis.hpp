#pragma once
#include <cstddef>
#include <string>
#include "ChassisMsg.hpp"


class IChassis{
    public:

    virtual ~IChassis() = default;
    virtual bool set_velocity(float linear, float angular) = 0;
    virtual void emergency_stop() = 0;
    virtual Wheel_Speed get_wheel_speed() = 0;

protected:
    IChassis() = default;
};


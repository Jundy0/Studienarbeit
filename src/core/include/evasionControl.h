#ifndef __EVASION_CONTROL_H__
#define __EVASION_CONTROL_H__

#include "lidar.h"
#include "vehicleControl.h"

class EvasionControl
{
public:
    EvasionControl(ILidar *lidar, IVehicleControl *vehicleControl);
    ~EvasionControl();
};

#endif //__EVASION_CONTROL_H__

#include "lidar.h"
#include "vehicleControl.h"
#ifndef __EVASION_CONTROL_H__
#define __EVASION_CONTROL_H__

class EvasionControl
{
public:
    EvasionControl(ILidar *lidar, IVehicleControl *vehicleControl);
    ~EvasionControl();
};

#endif //__EVASION_CONTROL_H__

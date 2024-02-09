#include <iostream>
#include <GL/glut.h>

#include "lidarSensorSim.h"
#include "vehicleActuatorSim.h"
#include "selfdrivingVehicle.h"

void display()
{
    glClear(GL_COLOR_BUFFER_BIT);

    glBegin(GL_POLYGON);
    glColor3f(1, 0, 0);
    glVertex3f(-0.6, -0.75, 0.5);
    glColor3f(0, 1, 0);
    glVertex3f(0.6, -0.75, 0);
    glColor3f(0, 0, 1);
    glVertex3f(0, 0.75, 0);
    glEnd();

    glFlush();
}

int main(int argc, char **argv)
{
    std::cout << "Test" << std::endl;
    LidarSensorSim lidarSensorSim = LidarSensorSim();

    lidarSensorSim.setPWM(3);
    lidarSensorSim.startScan();
    lidarSensorSim.stopScan();
    lidarSensorSim.getScanData((lidar_point_t *)0, 1000);

    VehicleActuatorSim vehicleActuatorSim = VehicleActuatorSim();

    SelfdrivingVehicle selfdrivingVehicle = SelfdrivingVehicle(&lidarSensorSim, &vehicleActuatorSim);

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);

    glutInitWindowPosition(80, 80);
    glutInitWindowSize(400, 300);
    glutCreateWindow("Self-driving Car Simulation");

    glutDisplayFunc(display);

    glutMainLoop();
}

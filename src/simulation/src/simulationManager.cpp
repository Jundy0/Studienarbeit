#include "simulationManager.h"

#include <GL/glut.h>
#include <iostream>

SimulationManager *SimulationManager::simulationManager = nullptr;

SimulationManager::SimulationManager(int argc, char **argv)
{
    simulationManager = this;

    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);

    glutInitWindowPosition(80, 80);
    glutInitWindowSize(600, 200);
    glutCreateWindow("Self-driving Car Simulation");

    glutDisplayFunc(this->display);
    glutMouseFunc(this->mouse);
    glutKeyboardFunc(this->keyboard);

    glutMainLoop();
}

SimulationManager::~SimulationManager()
{
}

void SimulationManager::display()
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

void SimulationManager::mouse(int button, int state, int x, int y)
{
    std::cout << "mouse clicked on " << x << ", " << y << "; button: " << button << std::endl;
}

void SimulationManager::keyboard(unsigned char key, int x, int y)
{
    std::cout << "key pressed " << (int)key << ": " << key << std::endl;
}

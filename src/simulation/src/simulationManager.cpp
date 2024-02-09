#include "simulationManager.h"

#include <GL/glut.h>

SimulationManager::SimulationManager(int argc, char **argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_SINGLE | GLUT_RGB);

    glutInitWindowPosition(80, 80);
    glutInitWindowSize(600, 200);
    glutCreateWindow("Self-driving Car Simulation");

    glutDisplayFunc([]()
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

    glFlush(); });

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
}

void SimulationManager::keyboard(unsigned char key, int x, int y)
{
}

#ifndef __SIMULATION_MANAGER_H__
#define __SIMULATION_MANAGER_H__

class SimulationManager
{
public:
    SimulationManager(int argc, char **argv);
    ~SimulationManager();

private:
    void display();
    void mouse(int button, int state, int x, int y);
    void keyboard(unsigned char key, int x, int y);
};

#endif // __SIMULATION_MANAGER_H__
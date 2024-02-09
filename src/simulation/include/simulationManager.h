#ifndef __SIMULATION_MANAGER_H__
#define __SIMULATION_MANAGER_H__

class SimulationManager
{
public:
    SimulationManager(int argc, char **argv);
    ~SimulationManager();

private:
    static SimulationManager *simulationManager;

    static void display();
    static void mouse(int button, int state, int x, int y);
    static void keyboard(unsigned char key, int x, int y);
};

#endif // __SIMULATION_MANAGER_H__

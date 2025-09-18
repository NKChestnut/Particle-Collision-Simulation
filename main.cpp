#include "simulator.h"

/*
1. Purpose
   Creates a small demo: 3 particles in a 10x10 box, 12 seconds of sim.

2. Notes
   - Adjust positions/velocities/radii/masses to explore different scenarios.
   - For visualization or logging, you can extend Simulator or add a tracer.
*/
int main() {
    SimConfig cfg;
    cfg.W = 10.0;
    cfg.H = 10.0;
    cfg.T_end = 12.0;
    cfg.max_events = 2000;
    cfg.enable_rollback = true;
    cfg.rollback_depth  = 8;

    std::vector<Particle> init = {
        Particle({2.0, 2.0}, { 1.2,  0.8}, 0.3, 1.0),
        Particle({5.5, 6.5}, {-0.9, -0.6}, 0.4, 1.5),
        Particle({7.8, 3.2}, {-0.4,  1.1}, 0.5, 2.0)
    };

    Simulator sim(cfg, init);
    sim.run();

    // Example: rollback one step and rerun (shows stack usage).
    // sim.undo();
    // sim.run();

    return 0;
}

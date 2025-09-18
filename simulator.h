#ifndef SIMULATOR_H
#define SIMULATOR_H

#include <vector>
#include <queue>
#include <stack>
#include <limits>
#include <iostream>
#include <iomanip>

#include "vec2.h"
#include "particle.h"
#include "event.h"

/*
1. Purpose
   Discrete-event simulation of 2D elastic collisions in a rectangular box.

2. Data Structures
   - priority_queue<Event, …, EventEarlier> : schedules future events by time
   - stack<SimState> : rollback snapshots (time + particle array)

3. Workflow
   a) Schedule initial wall and pair events from t = 0.
   b) Pop earliest event, drift system to event time, apply collision.
   c) Reschedule newly affected events (for impacted particles).
   d) Repeat until T_end or event budget reached.

4. Correctness Helpers
   - Stale-event invalidation via collision counters (coll_count).
*/

struct SimConfig {
    double W        = 10.0; // box width  (x in [0, W])
    double H        = 10.0; // box height (y in [0, H])
    double T_end    = 12.0; // simulation end time
    int    max_events = 2000;
    bool   enable_rollback = true;
    int    rollback_depth  = 8; // number of snapshots to retain
};

struct SimState {
    double               t;
    std::vector<Particle> P;
};

class Simulator {
public:
    // 1) Construction
    Simulator(const SimConfig& cfg, std::vector<Particle> init);

    // 2) Run simulation to cfg.T_end
    void run();

    // 3) Optional: rollback to a previous snapshot (reschedules events)
    bool undo();

private:
    // 4) Core helpers
    void snapshot();
    void schedule_all();
    void schedule_wall_events(int i);
    void schedule_pp_events_for(int i);
    bool valid(const Event& e) const;
    void drift_to(double T);

    // 5) Collision-time calculators
    double time_to_wall_x(const Particle& p) const;
    double time_to_wall_y(const Particle& p) const;
    double time_to_pp(const Particle& A, const Particle& B) const;

    // 6) Collision resolvers (elastic)
    void bounce_wall_x(int i);
    void bounce_wall_y(int i);
    void bounce_pp(int i, int j);

private:
    SimConfig cfg_;
    std::vector<Particle> P_;
    double t_ = 0.0;

    std::priority_queue<Event, std::vector<Event>, EventEarlier> pq_;
    std::stack<SimState> undo_;
};

#endif // SIMULATOR_H

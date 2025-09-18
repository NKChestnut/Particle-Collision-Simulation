#include "simulator.h"
#include <algorithm>
#include <cmath>

/*
1. Constructor
   Move-initialize particles and leave event heap empty until run().
*/
Simulator::Simulator(const SimConfig& cfg, std::vector<Particle> init)
    : cfg_(cfg), P_(std::move(init)) {}

/*
2. Snapshot
   Save (time, particle array) for rollback. Bounded by cfg_.rollback_depth.
*/
void Simulator::snapshot() {
    if (!cfg_.enable_rollback) return;
    if ((int)undo_.size() >= cfg_.rollback_depth) {
        // Trim by rebuilding stack (std::stack has no pop_bottom)
        std::vector<SimState> buf;
        buf.reserve(cfg_.rollback_depth);
        // Move all to buffer
        while (!undo_.empty()) { buf.push_back(undo_.top()); undo_.pop(); }
        // Remove the oldest (last in buffer)
        if (!buf.empty()) buf.pop_back();
        // Rebuild stack with newest on top
        for (auto it = buf.rbegin(); it != buf.rend(); ++it) undo_.push(std::move(*it));
    }
    undo_.push(SimState{t_, P_});
}

/*
3. Undo
   Restore last snapshot and reschedule all events from that state.
*/
bool Simulator::undo() {
    if (!cfg_.enable_rollback || undo_.empty()) return false;
    SimState s = undo_.top();
    undo_.pop();
    t_ = s.t;
    P_ = std::move(s.P);
    while (!pq_.empty()) pq_.pop();
    schedule_all();
    return true;
}

/*
4. Time Advancement
   Ballistic update of positions from current time to T.
*/
void Simulator::drift_to(double T) {
    double dt = T - t_;
    if (dt <= 0) return;
    for (auto& p : P_) p.r = p.r + p.v * dt;
    t_ = T;
}

/*
5. Collision-Time Helpers
   Return +inf if no future collision (or moving away).
*/
double Simulator::time_to_wall_x(const Particle& p) const {
    if (p.v.x > 0) return (cfg_.W - p.rad - p.r.x) / p.v.x;
    if (p.v.x < 0) return (p.rad - p.r.x) / p.v.x;
    return std::numeric_limits<double>::infinity();
}

double Simulator::time_to_wall_y(const Particle& p) const {
    if (p.v.y > 0) return (cfg_.H - p.rad - p.r.y) / p.v.y;
    if (p.v.y < 0) return (p.rad - p.r.y) / p.v.y;
    return std::numeric_limits<double>::infinity();
}

double Simulator::time_to_pp(const Particle& A, const Particle& B) const {
    Vec2 dr = B.r - A.r;
    Vec2 dv = B.v - A.v;
    const double R = A.rad + B.rad;

    const double dvdr = dv.dot(dr);
    if (dvdr >= 0) return std::numeric_limits<double>::infinity(); // separating

    const double dvdv = dv.norm2();
    const double drdr = dr.norm2();
    const double disc = dvdr * dvdr - dvdv * (drdr - R * R);
    if (disc < 0) return std::numeric_limits<double>::infinity();

    const double tcol = -(dvdr + std::sqrt(disc)) / dvdv;
    if (tcol <= 1e-12) return std::numeric_limits<double>::infinity();
    return tcol;
}

/*
6. Event Scheduling
   For current state/time, compute next wall/particle events and push to heap.
*/
void Simulator::schedule_wall_events(int i) {
    const auto& p = P_[i];
    double tx = time_to_wall_x(p);
    double ty = time_to_wall_y(p);

    if (std::isfinite(tx) && t_ + tx <= cfg_.T_end)
        pq_.push(Event(t_ + tx, i, -1, EventType::P_WALL_X, P_[i].coll_count, -1));
    if (std::isfinite(ty) && t_ + ty <= cfg_.T_end)
        pq_.push(Event(t_ + ty, i, -1, EventType::P_WALL_Y, P_[i].coll_count, -1));
}

void Simulator::schedule_pp_events_for(int i) {
    for (int j = i + 1; j < (int)P_.size(); ++j) {
        double dt = time_to_pp(P_[i], P_[j]);
        if (std::isfinite(dt) && t_ + dt <= cfg_.T_end) {
            pq_.push(Event(t_ + dt, i, j, EventType::P_P,
                           P_[i].coll_count, P_[j].coll_count));
        }
    }
}

void Simulator::schedule_all() {
    for (int i = 0; i < (int)P_.size(); ++i) {
        schedule_wall_events(i);
    }
    for (int i = 0; i < (int)P_.size(); ++i) {
        schedule_pp_events_for(i);
    }
}

/*
7. Event Validation
   If a particle's coll_count changed since scheduling, drop the event.
*/
bool Simulator::valid(const Event& e) const {
    if (e.a >= 0 && P_[e.a].coll_count != e.collA) return false;
    if (e.type == EventType::P_P && e.b >= 0 && P_[e.b].coll_count != e.collB) return false;
    return true;
}

/*
8. Collision Resolvers
   - Wall collisions reflect a single velocity component.
   - Particle collisions: elastic, along line-of-centers impulse.
*/
void Simulator::bounce_wall_x(int i) {
    P_[i].v.x = -P_[i].v.x;
    P_[i].coll_count++;
}

void Simulator::bounce_wall_y(int i) {
    P_[i].v.y = -P_[i].v.y;
    P_[i].coll_count++;
}

void Simulator::bounce_pp(int i, int j) {
    Particle& A = P_[i];
    Particle& B = P_[j];

    Vec2 dr = B.r - A.r;
    Vec2 dv = B.v - A.v;

    const double dist2 = dr.norm2();
    if (dist2 <= 0.0) return; // degenerate; skip

    // Project relative velocity onto the normal (line of centers).
    const double rel = dv.dot(dr) / dist2;

    // Impulse magnitude factor for unequal masses (1D along normal).
    const double mA = A.m, mB = B.m;
    Vec2 Jn = dr * rel; // direction along line of centers
    Vec2 impulse = Jn * (2.0 * mA * mB / (mA + mB));

    // Apply equal and opposite impulses.
    A.v = A.v + (impulse * (-1.0 / mA));
    B.v = B.v + (impulse * ( 1.0 / mB));

    A.coll_count++;
    B.coll_count++;
}

/*
9. Main Loop
   Pop validated events, advance, resolve, and reschedule effects.
*/
void Simulator::run() {
    schedule_all();

    int processed = 0;
    while (!pq_.empty() && processed < cfg_.max_events) {
        Event e = pq_.top();
        pq_.pop();

        if (e.t > cfg_.T_end) break;
        if (!valid(e)) continue;

        snapshot();      // for rollback/undo (optional)
        drift_to(e.t);   // advance to event time

        switch (e.type) {
            case EventType::P_WALL_X:
                bounce_wall_x(e.a);
                schedule_wall_events(e.a);
                schedule_pp_events_for(e.a);
                break;

            case EventType::P_WALL_Y:
                bounce_wall_y(e.a);
                schedule_wall_events(e.a);
                schedule_pp_events_for(e.a);
                break;

            case EventType::P_P:
                bounce_pp(e.a, e.b);
                // Reschedule events involving the two impacted particles.
                schedule_wall_events(e.a);
                schedule_wall_events(e.b);
                for (int k = 0; k < (int)P_.size(); ++k) {
                    if (k == e.a || k == e.b) continue;
                    // Pair (min, max) to avoid duplicates
                    int i1 = std::min(k, e.a), i2 = std::max(k, e.a);
                    double dt1 = time_to_pp(P_[i1], P_[i2]);
                    if (std::isfinite(dt1) && t_ + dt1 <= cfg_.T_end)
                        pq_.push(Event(t_ + dt1, i1, i2, EventType::P_P,
                                       P_[i1].coll_count, P_[i2].coll_count));

                    i1 = std::min(k, e.b); i2 = std::max(k, e.b);
                    double dt2 = time_to_pp(P_[i1], P_[i2]);
                    if (std::isfinite(dt2) && t_ + dt2 <= cfg_.T_end)
                        pq_.push(Event(t_ + dt2, i1, i2, EventType::P_P,
                                       P_[i1].coll_count, P_[i2].coll_count));
                }
                break;
        }
        processed++;
    }

    // drift remaining time if no more events
    drift_to(cfg_.T_end);

    // Print final state for quick verification.
    std::cout.setf(std::ios::fixed);
    std::cout << std::setprecision(4);
    std::cout << "Final Time: " << t_ << "\n";
    for (int i = 0; i < (int)P_.size(); ++i) {
        std::cout << "P" << i
                  << " r=(" << P_[i].r.x << "," << P_[i].r.y << ")"
                  << " v=(" << P_[i].v.x << "," << P_[i].v.y << ")"
                  << " collisions=" << P_[i].coll_count << "\n";
    }
}

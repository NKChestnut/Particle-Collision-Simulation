#ifndef PARTICLE_H
#define PARTICLE_H

#include "vec2.h"

/*
1. Purpose
   Represents a circular particle with position, velocity, radius, and mass.

2. Notes
   - coll_count increments on every collision to invalidate stale events.
*/
struct Particle {
    Vec2   r;// position
    Vec2   v;// velocity
    double rad;// radius
    double m;// mass
    int    coll_count;// collision counter for event validation

    Particle() : r(), v(), rad(0.5), m(1.0), coll_count(0) {}
    Particle(const Vec2& r_, const Vec2& v_, double rad_, double m_, int cc = 0)
        : r(r_), v(v_), rad(rad_), m(m_), coll_count(cc) {}
};

#endif // PARTICLE_H

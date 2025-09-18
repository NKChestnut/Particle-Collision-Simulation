#ifndef EVENT_H
#define EVENT_H

#include <functional>

/*
1. Purpose
   Defines event types and the Event struct used by the priority queue.

2. Event Validation
   We store the coll_count values at scheduling time; if they change by
   the time we pop from the queue, the event is stale and must be skipped.
*/

enum class EventType { P_WALL_X, P_WALL_Y, P_P };

struct Event {
    double    t;// absolute time when the event occurs
    int       a;// particle index A
    int       b;// particle index B (or -1 for wall events)
    EventType type;// event kind
    int       collA;// particle a collision count at schedule time
    int       collB;// particle b collision count at schedule time (or -1)

    Event() : t(0), a(-1), b(-1), type(EventType::P_WALL_X), collA(0), collB(-1) {}
    Event(double t_, int a_, int b_, EventType type_, int collA_, int collB_)
        : t(t_), a(a_), b(b_), type(type_), collA(collA_), collB(collB_) {}
};

// Comparator for min-heap by time (earlier events should come out first).
struct EventEarlier {
    bool operator()(const Event& lhs, const Event& rhs) const {
        return lhs.t > rhs.t; // reverse for std::priority_queue (min-heap)
    }
};

#endif // EVENT_H

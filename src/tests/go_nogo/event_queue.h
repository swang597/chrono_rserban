// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All right reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Author: Radu Serban
// =============================================================================
//
// A simple implementatin of a simulation event queue.  Events are functions
// that are triggered at a specified time.  Such functions can be defined
// using std::function/std::bind, lambdas, or functor classes which implement
// the operator().  The triggered function should have no arguments.
//
// The EventQueue class uses a priority queue, so events can be added in an
// arbitrary order.
//
// Sample usage:
//
//     void fun1(int i, double d) {
//         cout << i << "  " << d << endl;
//     }
//     
//     void fun2(const std::string& s) {
//         cout << s << endl;
//     }
//     
//     class Foo {
//       public:
//         void operator()() { cout << "Functor " << endl; }
//     };
//     
//     int main() {
//         EventQueue my_queue;
//         my_queue.AddEvent(2.5, std::bind(fun1, 2, 2.5));
//         my_queue.AddEvent(4.1, std::bind(fun1, 4, 4.3));
//         my_queue.AddEvent(4.7, [](){cout << "Lambda" << endl; });
//         my_queue.AddEvent(3.2, std::bind(fun2, "foo"));
//         my_queue.AddEvent(1.3, std::bind(fun1, 1, 1.3));
//         my_queue.AddEvent(2.7, Foo(115));
//     
//         for (double t = 0; t < 5; t += 0.11) {
//             my_queue.Process(t);
//         }
//     }
//
// =============================================================================

#ifndef GONOGO_EVENT_QUEUE_H
#define GONOGO_EVENT_QUEUE_H

#include <queue>
#include <functional>
#include <vector>

#define DEBUG_EVENT_QUEUE

class EventQueue {
  public:
    void AddEvent(double time, std::function<void()> const& function) { m_events.push(Event(time, function)); }

    void Process(double time) {
        if (m_events.empty()) {
#ifdef DEBUG_EVENT_QUEUE
            std::cout << "Queue empty at " << time << std::endl;
#endif
            return;
        }

        if (time < m_events.top().GetTime()) {
#ifdef DEBUG_EVENT_QUEUE
            std::cout << "Nothing to do at " << time << std::endl;
#endif
            return;
        }

#ifdef DEBUG_EVENT_QUEUE
        std::cout << "Trigger at " << time << std::endl;
#endif
        m_events.top().OnTrigger();
        m_events.pop();
    }

  private:
    class Event {
      public:
        Event(double time, std::function<void()> const& function) : m_time(time), m_function(function) {}

        double GetTime() const { return m_time; }
        void OnTrigger() {
#ifdef DEBUG_EVENT_QUEUE
            std::cout << "   Triggered " << m_time << "    ";
#endif
            m_function();
        }

      private:
        double m_time;
        std::function<void()> m_function;
    };

    class event_comparison {
      public:
        bool operator()(const Event& e1, const Event& e2) { return e1.GetTime() > e2.GetTime(); }
    };

    std::priority_queue<Event, std::vector<Event>, event_comparison> m_events;
};

#endif
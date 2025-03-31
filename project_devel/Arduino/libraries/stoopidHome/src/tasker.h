#ifndef TASKER_H
#define TASKER_H



#include <painlessMesh.h>
// #include "def.h"
#include "globals.h"

#pragma once

 const size_t MAX_TASKS = 10;
 const size_t MAX_SENSORs = 4;
class Tasker
{
    private:
        Scheduler &scheduler;
        Task *tasks[MAX_TASKS];  // Fixed-size array for tasks
        bool enabled[MAX_TASKS]; // Track enabled/disabled state
        size_t taskCount = 0;    // Number of tasks added

    public:
        Tasker(Scheduler &scheduler);

        size_t addTask(Task *t);
        bool enableTask(size_t tId);
        bool disableTask(size_t tId);
        bool run();
        Scheduler &getSchedulerRef();
};


#endif
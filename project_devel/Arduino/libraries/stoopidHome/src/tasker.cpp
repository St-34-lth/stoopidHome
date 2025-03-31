
#include "tasker.h"
#pragma once


Tasker::Tasker(Scheduler &scheduler) : scheduler(scheduler)
{
    for (size_t i = 0; i < MAX_TASKS; ++i)
    {
        tasks[i] = nullptr; // Initialize all task pointers to null
        enabled[i] = false; // Initialize all tasks as disabled
    };
};

size_t Tasker::addTask(Task *t)
{
    if (taskCount >= 10 || !t)
        return false; // Array is full
    tasks[taskCount] = t;
    scheduler.addTask(*t);
    return taskCount++;
};

bool Tasker::enableTask(size_t tId)
{
    if (!tasks[tId] || tId >= taskCount)
    {
        return false;
    }
    tasks[tId]->enable();
    enabled[tId] = true;
    return true;
};

bool Tasker::disableTask(size_t tId)
{

    if (!tasks[tId])
    {
        return false;
    }
    tasks[tId]->disable();
    enabled[tId] = false;
    return true;
};

bool Tasker::run()
{

    scheduler.execute();
    return true;
};

Scheduler &Tasker::getSchedulerRef()
{
    return scheduler;
}
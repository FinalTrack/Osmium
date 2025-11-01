#pragma once
#include <thread>
#include <vector>
#include <atomic>
#include <barrier>
#include <chrono>
#include "World.hpp"

struct Engine
{
    World* world;
    int nThreads;
    std::atomic<bool> stopFlag{false};

    enum class TaskType { Gather, SAT };
    struct Task { TaskType type; int t, a, b; };

    std::vector<std::thread> workers;
    std::vector<std::vector<std::pair<int,int>>> results;
    std::vector<std::vector<Task>> tasks;

    std::barrier<> startBarrier;
    std::barrier<> finishBarrier;

    Engine(int threadCount, World* w)
        : world(w),
          nThreads(threadCount),
          startBarrier(nThreads + 1),
          finishBarrier(nThreads + 1)
    {
        tasks.resize(nThreads);
        results.resize(nThreads);
        for (int i = 0; i < nThreads; ++i)
            workers.emplace_back([this, i]{ workerLoop(i); });
    }

    ~Engine() {
        stopFlag = true;
        startBarrier.arrive_and_wait();
        finishBarrier.arrive_and_wait();
        for (auto& t : workers) if (t.joinable()) t.join();
    }

    void updateStep(float dt, float& tu, float& tc, float& tr)
    {
        using clock = std::chrono::high_resolution_clock;
        auto t0 = clock::now();

        world->updateVelocities(dt);
        world->updatePositions(dt);
        world->initGrid();
        auto t1 = clock::now();

        // Phase 1: Broadphase
        clearTasks();
        int cur = 0;
        for(int id = 0; id < world->allocated; id++)
            if (world->bodies[id].active)
                tasks[cur++ % nThreads].push_back({TaskType::Gather, -1, id, -1});

        startBarrier.arrive_and_wait();
        finishBarrier.arrive_and_wait();

        world->collisionPairs.clear();
        for (auto& r : results) 
            world->collisionPairs.insert(world->collisionPairs.end(), r.begin(), r.end());
        auto t2 = clock::now();

        // Phase 2: Narrowphase
        int N = (int)world->collisionPairs.size();
        world->collisionData.resize(N);
        clearTasks();
        for (int i = 0; i < N; ++i)
        {
            auto [a,b] = world->collisionPairs[i];
            tasks[i % nThreads].push_back({TaskType::SAT, i, a, b});
        }

        startBarrier.arrive_and_wait();
        finishBarrier.arrive_and_wait();

        world->resolveCollisions();
        world->applyCorrections();
        world->resetGrid();
        auto t3 = clock::now();

        tu = std::chrono::duration<float, std::micro>(t1 - t0).count();
        tc = std::chrono::duration<float, std::micro>(t2 - t1).count();
        tr = std::chrono::duration<float, std::micro>(t3 - t2).count();
    }

    void workerLoop(int i)
    {
        auto& myTasks = tasks[i];
        auto& myResults = results[i];
        while (!stopFlag)
        {
            startBarrier.arrive_and_wait();
            // After main thread reaches start barrier, we can execute the tasks in parallel
            if (stopFlag) { finishBarrier.arrive_and_wait(); break; }

            myResults.clear();
            for (auto& t : myTasks)
            {
                if (t.type == TaskType::Gather)
                    world->getNeighbors(t.a, myResults);
                else
                    world->collisionData[t.t] = Body::performSAT(world->bodies[t.a], world->bodies[t.b]);
            }
            // Signal to the main thread that this worker thread has completed its task
            finishBarrier.arrive_and_wait();
        }
    }

    void clearTasks()
    {
        for (auto& t : tasks) t.clear();
        for (auto& r : results) r.clear();
    }
};

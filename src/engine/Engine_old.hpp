#pragma once

#include <thread>
#include <functional>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <cassert>
#include "World.hpp"
#include <chrono>
#include <iostream>

struct Barrier
{
    std::mutex mtx;
    std::condition_variable cv;
    int cnt;
    int threshold;
    int generation;

    Barrier(int thresh)
    {
        threshold = thresh;
        cnt = 0;
        generation = 0;
    }

    void wait()
    {
        std::unique_lock<std::mutex> lk(mtx);
        int gen = generation;
        if(++cnt == threshold)
        {
            generation ^= 1;
            cnt = 0;
            cv.notify_all();
        }
        else
            cv.wait(lk, [&]{ return gen != generation; });
    }
};

struct Engine
{
    Barrier start, done;
    int threadCnt;
    World* world;

    std::vector<std::vector<std::array<int, 3>>> tasks;
    std::vector<std::vector<std::pair<int, int>>> results;
    std::vector<std::thread> workers;
    std::atomic<bool> stopFlag;

    Engine(int N, World* w) : start(N+1), done(N+1), threadCnt(N) , world(w), stopFlag(false)
    {
        workers.reserve(N);
        tasks.resize(N);
        results.resize(N);
        for (int i = 0; i < N; ++i) {
            workers.emplace_back([this,i]{
                while (true) {
                    start.wait();
                    if(stopFlag)
                        break;
                    for(auto [t, u, v]: tasks[i])
                    {
                        if(t == -1)
                            world->getNeighbors(u, results[i]);
                        else
                        world->collisionData[t] = Body::performSAT(world->bodies[u], world->bodies[v]);
                    }
                    done.wait();
                }
            });
        }
    }

    void updateStep(float DT, float& tu, float& tc, float& tr)
    {
        auto t0 = std::chrono::high_resolution_clock::now();
        world->updateVelocities(DT);
        world->updatePositions(DT);
        world->initGrid();
        auto t1 = std::chrono::high_resolution_clock::now();

        world->collisionPairs.clear();
        for(int i = 0; i < threadCnt; i++)
            tasks[i].clear();
        for(int i = 0; i < threadCnt; i++)
           results[i].clear();
        int curr = 0;
        for(int id = 0; id < world->allocated; id++)
        {
            if(world->bodies[id].active)
            {
                tasks[curr].push_back({-1, id, 0});
                curr = (curr + 1) % threadCnt;
            }
        }

        start.wait();
        done.wait();

        for(int i = 0; i < threadCnt; i++)
        {
            for(auto [id1, id2]: results[i])
                world->collisionPairs.emplace_back(id1, id2);
        }

        auto t2 = std::chrono::high_resolution_clock::now();

        int sz = world->collisionPairs.size();
        world->collisionData.resize(sz);
        for(int i = 0; i < threadCnt; i++)
            tasks[i].clear();

        curr = 0;
        for(int i = 0; i < sz; ++i)
        {
            auto [id1, id2] = world->collisionPairs[i];
            tasks[curr].push_back({i, id1, id2});
            curr = (curr + 1) % threadCnt;
        }

        start.wait();
        done.wait();

        world->resolveCollisions();
        world->applyCorrections();
        world->resetGrid();
        auto t3 = std::chrono::high_resolution_clock::now();

        tu = std::chrono::duration<float, std::micro>(t1 - t0).count();
        tc = std::chrono::duration<float, std::micro>(t2 - t1).count();
        tr = std::chrono::duration<float, std::micro>(t3 - t2).count();
    }

    ~Engine() {
        stopFlag = true;
        start.wait();
        for (auto &t : workers) t.join();
    }
};
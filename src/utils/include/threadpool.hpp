#pragma once

#include <thread>
#include <vector>
#include <functional>
#include <deque>
#include <queue>
#include <mutex>
#include <condition_variable>
#include <iostream>

class ThreadPool {
public:
    ThreadPool(const int& num_threads = std::thread::hardware_concurrency()) {
        for (size_t i = 0; i < num_threads; ++i) {
            m_threads.emplace_back([this]() {
                while (true) {
                    std::function<void()> task;
                    {
                        std::unique_lock<std::mutex> lock(m_queue_mutex);

                        m_condv.wait(lock, [this] {
                            return !m_task_queue.empty() || m_stop;
                        });

                        if (m_stop && m_task_queue.empty()) {
                            return;
                        }
                        task = std::move(m_task_queue.front());
                        m_task_queue.pop();
                        // std::cout << "task put on Thread id: " << std::this_thread::get_id() << "\n";
                    }

                    task();
                }
            });
        }
    }
    ~ThreadPool() {
        {
            std::unique_lock<std::mutex> lock(m_queue_mutex);
            m_stop = true;
        }

        m_condv.notify_all();
        for (auto &thread : m_threads) {
            if (thread.joinable()) {
                thread.join();
            }
        }
    }

    void queue_task(std::function<void()> task) {
        {
            std::unique_lock<std::mutex> lock(m_queue_mutex);
            m_task_queue.emplace(task);
            // std::cout << "Task added to the queue..\n";
        }
        m_condv.notify_one();
    }
private:
    std::queue<std::function<void()>> m_task_queue;
    std::vector<std::thread> m_threads;

    std::mutex m_queue_mutex;
    std::condition_variable m_condv;
    bool m_stop = false;
};
#ifndef __MUX_HPP__
#define __MUX_HPP__

#include <chrono>
#include <functional>
#include <mutex>
#include <vector>

#include "dds_publisher.hpp"
#include "dds_subscriber.hpp"

template <typename T>
class DDSMux {
   public:
    DDSMux(const std::string& outputTopic,
           const std::vector<std::string>& inputTopics,
           std::function<int(const T&)> priorityExtractor, int domainId = 0)
        : publisher(outputTopic, domainId),
          priorityExtractor(priorityExtractor),
          lastReceivedTimes(inputTopics.size(),
                            std::chrono::steady_clock::time_point::min()),
          timeout(std::chrono::seconds(1)) {
        for (size_t i = 0; i < inputTopics.size(); ++i) {
            subscribers.push_back(std::make_shared<DDSSubscriber<T>>(
                inputTopics[i],
                [this, i](const T& data) {
                    std::lock_guard<std::mutex> lock(
                        mutex);  // Lock for thread safety
                    lastReceivedTimes[i] = std::chrono::steady_clock::now();
                    // this->update();  // Process immediately on new message
                },
                domainId));
        }
    }

    void update() {
        std::lock_guard<std::mutex> lock(mutex);  // Ensure thread-safe access
        auto now = std::chrono::steady_clock::now();
        T selectedMessage = T();  // Default message
        int maxPriority = -1;
        bool found = false;

        for (size_t i = 0; i < subscribers.size(); ++i) {
            auto timeSinceLast = now - lastReceivedTimes[i];
            if (timeSinceLast < timeout) {  // Consider only recent messages
                T msg = subscribers[i]->getLatestMessage();
                int priority = priorityExtractor(msg);
                if (priority > maxPriority) {
                    maxPriority = priority;
                    selectedMessage = msg;
                    found = true;
                }
            }
        }

        if (found && maxPriority > 0) {
            publisher.publish(selectedMessage);
        } else {
            publisher.publish(T());  // Publish default if no valid messages
        }
    }

   private:
    DDSPublisher<T> publisher;
    std::vector<std::shared_ptr<DDSSubscriber<T>>> subscribers;
    std::function<int(const T&)> priorityExtractor;
    std::mutex mutex;
    std::vector<std::chrono::steady_clock::time_point> lastReceivedTimes;
    std::chrono::steady_clock::duration timeout;
};

#endif

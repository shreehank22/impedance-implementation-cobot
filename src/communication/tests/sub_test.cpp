/*
 * Copyright(c) 2006 to 2020 ZettaScale Technology and others
 *
 * This program and the accompanying materials are made available under the
 * terms of the Eclipse Public License v. 2.0 which is available at
 * http://www.eclipse.org/legal/epl-2.0, or the Eclipse Distribution License
 * v. 1.0 which is available at
 * http://www.eclipse.org/org/documents/edl-v10.php.
 *
 * SPDX-License-Identifier: EPL-2.0 OR BSD-3-Clause
 */
#include <cstdlib>
#include <iostream>
#include <chrono>
#include <thread>

#include "dds_subscriber.hpp"
#include "String.hpp"

using namespace org::eclipse::cyclonedds;
using namespace std_msgs::msg::dds_;

void message_callback(const String_& msg) {
    std::cout << "Received message: " << msg.data() << std::endl;
}

int main() {
    try {
        std::cout << "=== [Subscriber] Create reader." << std::endl;

        DDSSubscriber<String_> test_sub("rt/chatter", message_callback);

        while(true) {
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    }
    catch (const dds::core::Exception& e) {
        std::cerr << "=== [Subscriber] Exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "=== [Subscriber] Done." << std::endl;

    return EXIT_SUCCESS;
}
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

#include "dds_publisher.hpp"
#include "String.hpp"

using namespace org::eclipse::cyclonedds;
using namespace std_msgs::msg::dds_;

int main() {
    try {
        std::cout << "=== [Publisher] Create writer." << std::endl;

        DDSPublisher<String_> test_pub("rt/chatter");

        while(true) {
            String_ hello = String_();
            hello.data() = "hello!";
            test_pub.publish(hello);
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
        }
    }
    catch (const dds::core::Exception& e) {
        std::cerr << "=== [Publisher] Exception: " << e.what() << std::endl;
        return EXIT_FAILURE;
    }

    std::cout << "=== [Publisher] Done." << std::endl;

    return EXIT_SUCCESS;
}
/*
Ping command test
This tests whether a servo with a specific ID is responding on the bus.
Broadcast ping is only recommended if only one servo is on the bus.
*/

#include <iostream>
#include "SCServo.h"

SMS_STS sm_st;

const int commonBaudRates[] = {1000000, 115200, 57600, 38400, 100000};
const int numBaudRates = sizeof(commonBaudRates) / sizeof(commonBaudRates[0]);

int main(int argc, char **argv)
{
    if(argc < 2) {
        std::cout << "Usage: " << argv[0] << " <serial port>" << std::endl;
        return 0;
    }

    const char* serialPort = argv[1];
    std::cout << "Trying serial port: " << serialPort << std::endl;

    bool found = false;

    for(int b = 0; b < numBaudRates; ++b) {
        int baud = commonBaudRates[b];
        std::cout << "\nTrying baud rate: " << baud << std::endl;

        if(!sm_st.begin(baud, serialPort)) {
            std::cout << "Failed to initialize at baud " << baud << std::endl;
            continue;
        }

        // Scan IDs from 0 to 252
        for(int id = 0; id <= 252; ++id) {
            int result = sm_st.Ping(id);
            if(result != -1) {
                std::cout << "✔ Found servo at ID: " << id << " (baud: " << baud << ")" << std::endl;
                found = true;
                // Uncomment below if you only want to find the first one
                // sm_st.end();
                // return 0;
            }
        }

        sm_st.end();
    }

    if(!found) {
        std::cout << "\n❌ No servos found on port " << serialPort << " at common baud rates." << std::endl;
    }

    return 0;
}

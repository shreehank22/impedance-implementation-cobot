#ifndef RATE_LIMITED_PRINTER_H
#define RATE_LIMITED_PRINTER_H

#include <string>
#include <iostream>

class RateLimitedPrinter {
    private:
        unsigned int counter;
        unsigned int N;

    public:
        inline RateLimitedPrinter(unsigned int rate_divider = 100)
            :counter(0), N(rate_divider) {}
        
        inline void print(const std::string& message) {
            if (counter % N == 0) {
                std::cout << message << std::endl;
            }
            counter++;

        if (counter >= 1000000) {
            counter = 0;
        }
    }
};

#endif
#pragma once

#include <deque>
#include <numeric>
#include "Filter.hpp"

class MovingAverageFilter : public Filter {
public:
    MovingAverageFilter(const double& samplePeriod, const double& cutoffFrequency);
    MovingAverageFilter(const double& windowSize);
    ~MovingAverageFilter() {}
protected:
    void updateEstimate(const double& newValue) override;
private:
    int m_window_size;
    std::deque<double> m_value_history;
};
#include "MovingAverageFilter.hpp"

MovingAverageFilter::MovingAverageFilter(const double& samplePeriod, const double& cutoffFrequency) : Filter(samplePeriod, cutoffFrequency)  {
    m_window_size = 1. / (m_cutoffFrequency * m_samplePeriod);
}

MovingAverageFilter::MovingAverageFilter(const double& windowSize) : m_window_size(windowSize), Filter(1, windowSize) {}

void MovingAverageFilter::updateEstimate(const double& newValue) {
    if (m_value_history.size() > m_window_size) {
        m_value_history.pop_front();
    }
    m_value_history.push_back(newValue);
    m_filtered_value = std::accumulate(m_value_history.begin(), m_value_history.end(), 0.) / m_window_size;
}
#include "Filter.hpp"
#include <math.h>

Filter::Filter(const double& samplePeriod, const double& cutoffFrequency): m_samplePeriod(samplePeriod), m_cutoffFrequency(cutoffFrequency) {
}

void Filter::addValue(const double& newValue) {
    if(!m_start) {
        m_start = true;
        m_filtered_value = newValue;
        return;
    }
    updateEstimate(newValue);
}

void Filter::updateEstimate(const double& newValue) {
    // Identity filter
    m_filtered_value = newValue;
}

double Filter::getValue() {
    return m_filtered_value;
}

void Filter::clear(){
    m_start = false;
}
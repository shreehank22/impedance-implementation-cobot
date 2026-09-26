#include "LowPassFilter.hpp"
#include <math.h>

LPFilter::LPFilter(const double& samplePeriod, const double& cutoffFrequency) : Filter(samplePeriod, cutoffFrequency) {
    initClass();
}

void LPFilter::initClass() {
    m_alpha = 1.0 / ( 1.0 + 1.0 / (2.0 * M_PI * m_samplePeriod * m_cutoffFrequency) );
}

void LPFilter::updateEstimate(const double& newValue) {
    m_filtered_value = m_alpha * newValue + (1 - m_alpha) * m_filtered_value;
}
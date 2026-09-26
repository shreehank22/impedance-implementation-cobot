#include "IIRFilter.hpp"
#include <math.h>

IIRFilter::IIRFilter(const double& samplePeriod, const double& cutoffFrequency) : Filter(samplePeriod, cutoffFrequency) {
    initClass();
}

void IIRFilter::initClass() {
    m_filter.setup(1. / m_samplePeriod, m_cutoffFrequency);
}

void IIRFilter::updateEstimate(const double& newValue) {
    m_filtered_value = m_filter.filter(newValue);
}
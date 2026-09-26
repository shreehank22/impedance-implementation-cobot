#include "AccLimitFilter.hpp"
#include <math.h>

#include "utils.hpp"

AccLimitFilter::AccLimitFilter(const double& samplePeriod, const double& accLimit) : Filter(samplePeriod, 0.), 
                                                                                     m_acc_limit(accLimit) {
    initClass();
}

void AccLimitFilter::initClass() {
    m_kd = 50;
    m_acc = m_vel = 0;
}

void AccLimitFilter::updateEstimate(const double& newValue) {
    m_acc = m_kd * (newValue - m_filtered_value);
    m_acc = saturate(m_acc, -m_acc_limit, m_acc_limit);
    m_filtered_value += m_acc * m_samplePeriod;
}
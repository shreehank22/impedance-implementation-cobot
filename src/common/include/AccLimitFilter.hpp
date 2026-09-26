#pragma once

#include "Filter.hpp"

class AccLimitFilter : public Filter {
public:
    AccLimitFilter(const double& samplePeriod, const double& accLimit);
    ~AccLimitFilter() {}
protected:
    void updateEstimate(const double& newValue) override;
private:
    void initClass();
    double m_acc = 0;
    double m_vel = 0;
    double m_acc_limit = 0;
    double m_kd = 10;
};
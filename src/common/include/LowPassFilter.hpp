#pragma once

#include "Filter.hpp"

class LPFilter : public Filter {
public:
    LPFilter(const double& samplePeriod, const double& cutoffFrequency);
    ~LPFilter() {}
protected:
    void updateEstimate(const double& newValue) override;
private:
    void initClass();
    double m_alpha;
};
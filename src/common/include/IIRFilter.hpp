#pragma once

#include "Filter.hpp"
#include "Iir.h"

class IIRFilter : public Filter {
public:
    IIRFilter(const double& samplePeriod, const double& cutoffFrequency);
    ~IIRFilter() {}
protected:
    void updateEstimate(const double& newValue) override;
private:
    void initClass();
    double m_alpha;
    Iir::Butterworth::LowPass<2> m_filter;
};
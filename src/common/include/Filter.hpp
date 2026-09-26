#pragma once

class Filter {
public:
    Filter(const double& samplePeriod, const double& cutoffFrequency);
    ~Filter() {}
    void addValue(const double& newValue);
    double getValue();
    void clear();
protected:
    virtual void updateEstimate(const double& newValue);
    double m_filtered_value = 0;
    double m_samplePeriod = 0;
    double m_cutoffFrequency = 0;
private:
    bool m_start = false;

};
#pragma once

#include "CommunicationManager.hpp"
#include <sys/ipc.h>
#include <sys/shm.h>

#include "dds_subscriber.hpp"
#include "JoyData.hpp"

using namespace org::eclipse::cyclonedds;
using namespace xterra::msg::dds_;

// Randomy chosen key enums
enum SHM_KEYS
{
    SENSOR_DATA = 123,
    MEASUREMENT_DATA = 496,
    COMMAND_DATA = 549,
    PLANT_TIME = 837
};

class SHM : public CommunicationManager
{
public:
    SHM();
    SHM(const DATA_ACCESS_MODE &mode);
    SHM(const std::string &name, const DATA_ACCESS_MODE &mode);
    ~SHM();

    // void writeSensorData(const CobotSensorData& sensor_data) override;
    // void writeCommandData(const CobotCommandData& cmd_data) override;

    void setPlannerDataPtr(CobotPlannerData *planner_data_ptr) override {}

    void setSensorDataPtr(CobotSensorData *sensor_data_ptr) override {}

    void setCommandDataPtr(CobotCommandData *command_data_ptr) override {}

    // void setMeasurementDataPtr(CobotMeasurementData *measurement_data_ptr) override {}

    void setEstimationDataPtr(CobotEstimationData *estimation_data_ptr) override {}

    // void setJoystickDataPtr(CobotJoystickData *joystick_data_ptr) override {}
    void setCliDataPtr(CobotCliData *cli_data_ptr) override {}

    void setPlantTimePtr(double *time_ptr) override {}

private:
    std::string m_name;



    void initClass();

    void SetupMemory();


    void *getSHMPointer(const key_t &, const size_t &);
};
#include "StateMachine/FSM.hpp"

FSM::FSM(std::shared_ptr<RobotComponents> robotComp, const YAML::Node& config)
    : m_robotComp(std::move(robotComp)), m_config(config)
{
    m_stateList.invalid = nullptr;
    m_stateList.cobot_sleep = std::make_shared<StateCobotSleep>(m_robotComp, m_config["cobot_sleep"]);
    m_stateList.cobot_p2p = std::make_shared<StateCobotP2P>(m_robotComp, m_config["cobot_p2p"]);
    m_stateList.cobot_teleop=
        std::make_shared<StateCobotTeleop>(m_robotComp, m_config["cobot_teleop"]);
    initiate();
}

FSM::~FSM()
{
    updateTime();
    m_currentState->exit();
    m_currentState = m_stateList.cobot_sleep;
    m_currentState->setTimeOfEntry(t_curr);
    m_currentState->enter();
    t_switch = t_curr;

    uint8_t status = 0;

    while (!status)
    {
        status = m_currentState->run();
        updateTime();

        stepFSM();

        if (use_us_wait)
            us_wait();
        else
            ms_wait();
    }

    m_currentState->exit();
}

void FSM::initiate()
{
    m_currentState = m_stateList.cobot_sleep;
    m_nextState = m_currentState;
    m_mode = FSMMode::NORMAL;
}

void FSM::updateTime()
{
    if (m_robotComp->m_first_run)
    {
        m_robotComp->m_first_run = false;
        t_last = t_curr = get_wall_time_seconds(m_robotComp->m_startTimePoint);
    }
    else
    {
        t_last = t_curr;
        if (m_robotComp->use_plant_time)
        {
            m_robotComp->getCommunicationManager()->getPlantTime(t_curr);
        }
        else
        {
            t_curr = get_wall_time_seconds(m_robotComp->m_startTimePoint);
        }
        m_robotComp->m_dt = t_curr - t_last;
    }
}

void FSM::ms_wait()
{
    double t_end = 0;
    if (m_robotComp->use_plant_time)
    {
        m_robotComp->getCommunicationManager()->getPlantTime(t_end);
    }
    else
    {
        t_end = get_wall_time_seconds(m_robotComp->m_startTimePoint);
    }

    int delay_time = delay_ms - (t_end - t_curr) * 1e3;
    if (delay_time > 0)
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(delay_time));
    }
}

void FSM::us_wait()
{
    auto t_end = std::chrono::high_resolution_clock::now();
    
    if (m_robotComp->use_plant_time)
    {
        double plant_time;
        m_robotComp->getCommunicationManager()->getPlantTime(plant_time);
        t_end = m_robotComp->m_startTimePoint + 
                std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
                    std::chrono::duration<double>(plant_time));
    }
    
    m_next_wake_time = m_robotComp->m_startTimePoint + 
                       std::chrono::duration_cast<std::chrono::high_resolution_clock::duration>(
                           std::chrono::duration<double>(t_curr) + m_period);
    
    auto time_remaining = m_next_wake_time - t_end;
    auto remaining_us = std::chrono::duration_cast<std::chrono::microseconds>(time_remaining).count();

    if (time_remaining.count() > 0)
    {
        std::this_thread::sleep_until(m_next_wake_time);
    }
}

void FSM::stepFSM()
{
    if (std::fabs(t_curr - t_last) > 1e-6) {
        m_dt = t_curr - t_last;
        m_robotComp->step(m_dt, t_curr - t_switch);
    }
}

void FSM::run(const bool &terminate)
{
    m_robotComp->getCommunicationManager()->setExecutorReady(false);

    if (m_robotComp->m_rate == 0)
    {
        std::cout << "Update rate not set. Defaulting to 250 Hz!\n";
        m_robotComp->m_rate = 250;
        m_robotComp->m_dt = 1. / 250;
        m_dt = 1. / 250;
    }
    m_dt = m_robotComp->m_dt;
    delay_ms = m_robotComp->m_dt * double(1e3);
    m_period = std::chrono::microseconds(static_cast<int64_t>(m_robotComp->m_dt * 1e6));

    m_robotComp->m_startTimePoint = std::chrono::high_resolution_clock::now();
    m_next_wake_time = m_robotComp->m_startTimePoint;
    m_robotComp->runCalibration();

    if (!m_robotComp->getCommunicationManager()->isReady()) {
        std::cerr << "[FSM] No sensor data received. Communication failure. Exiting...\n";
        return;
    }

#ifdef USE_HARDWARE
    vec6 q_sleep;
    q_sleep << 0.001, -0.001, 0.001, 0, 0, 0;

    if ((m_robotComp->getSensorData().q - q_sleep).norm() > 0.5) {
        std::cout << "q: " << m_robotComp->getSensorData().q.transpose() << "\n";
        std::cout << "q_sleep: " << q_sleep.transpose() << "\n";
        std::cerr << "[FSM] Joint angles not in the starting sleep position. Exiting...\n";
        return;
    }
#endif

    m_currentState->enter();

    while (true && !terminate)
    {
        updateTime();

        if (m_mode == FSMMode::NORMAL)
        {
            uint8_t status = m_currentState->run();

            m_nextStateName = m_currentState->checkChange();
            if (m_nextStateName != m_currentState->m_stateName)
            {
                m_mode = FSMMode::TRANSITION;
                m_nextState = getNextState(m_nextStateName);
                std::cout << "[FSM] Switched from " << m_currentState->m_stateNameString
                          << " to " << m_nextState->m_stateNameString << std::endl;
            }
        }
        else if (m_mode == FSMMode::TRANSITION)
        {
            m_currentState->exit();
            m_currentState = m_nextState;
            m_currentState->setTimeOfEntry(t_curr);
            m_currentState->enter();
            m_mode = FSMMode::NORMAL;
            m_currentState->run();
            t_switch = t_curr;
        }

        stepFSM();

        if (use_us_wait)
            us_wait();
        else
            ms_wait();
    }
}

std::shared_ptr<FSMState> FSM::getNextState(FSMStateName stateName)
{
    switch (stateName)
    {
    case FSMStateName::COBOTSLEEP:
        return m_stateList.cobot_sleep;
        break;
    case FSMStateName::COBOTP2P:
        return m_stateList.cobot_p2p;
        break;
    case FSMStateName::COBOTTELEOP:
        return m_stateList.cobot_teleop;
        break;
    case FSMStateName::INVALID:
    default:
        return nullptr;
    }
}
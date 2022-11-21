#ifndef TELE_STATE_MACHINE_HPP_
#define TELE_STATE_MACHINE_HPP_

#include <memory>
#include <string>
#include <iostream>

// Autoware msg
#include <autoware_auto_system_msgs/msg/emergency_state.hpp>
#include <autoware_auto_system_msgs/msg/autoware_state.hpp>

// teleoperation msgs
#include <scenery_msgs/msg/tele_state.hpp>

// ROS2 core
#include <rclcpp/create_timer.hpp>
#include <rclcpp/rclcpp.hpp>

using scenery_msgs::msg::teleState;
using autoware_auto_system_msgs::msg::EmergencyState;
using autoware_auto_system_msgs::msg::AutowareState;

enum TeleState : uint8_t{
    WaitingForEngagement,
    Driving,
    RequestingForTakingOver,
    Teleoperation,
};
class TeleStateMachine
{
public:
    TeleStateMachine(){std::cout << "State Machine Initialized!\n";}
    ~TeleStateMachine() = default;
    inline TeleState getCurrentState() const {return currentState;}
    inline void setCurrentState(const TeleState &inputState) {currentState = inputState;}
    void checkAndSetCurrentState(const EmergencyState::ConstSharedPtr msg);
    void checkAndSetCurrentState(const AutowareState::ConstSharedPtr msg);
    bool isWaitingForEngagement() const;
    bool isDriving() const;
    bool isTakeoverRequest() const;
    bool isTeleoperation() const;
private :
    TeleState currentState{TeleState::WaitingForEngagement};
    
};

#endif // TELE_STATE_MACHINE_HPP_
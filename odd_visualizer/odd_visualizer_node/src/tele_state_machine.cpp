#include "tele_state_machine.hpp"

void TeleStateMachine::checkAndSetCurrentState(const EmergencyState::ConstSharedPtr msg) {
    std::cout << "emergency state: " << msg->state << '\n';
    if (isDriving() && msg->state == EmergencyState::OVERRIDE_REQUESTING) {
        setCurrentState(TeleState::RequestingForTakingOver);
    }
}

void TeleStateMachine::checkAndSetCurrentState(const AutowareState::ConstSharedPtr msg) {
    if (isWaitingForEngagement() && msg->state == AutowareState::DRIVING) {
        setCurrentState(TeleState::Driving);
    }
}

bool TeleStateMachine::isWaitingForEngagement() const{
    // in case there are more conditions to determin this state, here
    // apply "not if" to check if the current state is "WaitingForEngagement"
    if (currentState != TeleState::WaitingForEngagement)
        return false;
    return true;
}

bool TeleStateMachine::isDriving() const {
    if (currentState != TeleState::Driving)
        return false;
    return true;
}


bool TeleStateMachine::isTakeoverRequest() const {
    if (currentState != TeleState::RequestingForTakingOver)
        return false;
    return true;
}


bool TeleStateMachine::isTeleoperation() const {
    if (currentState != TeleState::Teleoperation)
        return false;
    return true;
}

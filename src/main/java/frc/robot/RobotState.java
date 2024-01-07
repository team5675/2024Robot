package frc.robot;

import java.util.Optional;

public class RobotState {
 
    public enum Action {
        DRIVING,
        INTAKING,
        AIMING,
        LAUNCHING,
        PATHING,
        CLIMBING,
        IDLE
    }

    Action currentState;
    Optional<Action> desiredState;

    public RobotState() {
        currentState = Action.IDLE;
        desiredState = Optional.empty();
    }
}

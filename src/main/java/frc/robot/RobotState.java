package frc.robot;

import java.util.Optional;

public class RobotState {
 
    public enum State {
        DRIVING,
        INTAKING,
        AIMING,
        LAUNCHING,
        PATHING,
        CLIMBING,
        IDLE
    }

    public enum Event {

        INTAKE_REQUEST,
        INTAKE_CANCEL,
        INTAKE_PROX,
        LAUNCHER_PROX,
        AIMING_COMPLETE,
        LAUNCHER_SHOT,
        DRIVING_REQUEST,
        PATHING_REQUEST;
    }

    State currentState;
    Optional<State> desiredState;

    Optional<Event> mostRecentEvent;

    boolean validTransition;

    public RobotState() {
        currentState = State.IDLE;
        desiredState = Optional.empty();

        mostRecentEvent = Optional.empty();
    }

    public void periodic() {

        if(mostRecentEvent.isPresent()) {
            switch (mostRecentEvent.get()) {
                case INTAKE_REQUEST:
                    
                    break;
                
                case INTAKE_CANCEL:
                    
                    break;

                case INTAKE_PROX:
                    
                    break;

                case LAUNCHER_PROX:
                    
                    break;

                case AIMING_COMPLETE:
                    
                    break;

                case LAUNCHER_SHOT:
                    
                    break;

                case DRIVING_REQUEST:
                    
                    break;

                case PATHING_REQUEST:
                    
                    break;
            
                default:
                    break;
            }
        }
    }

    public Event getEvent() {
        return Event.DRIVING_REQUEST;
    }

    public boolean isValidTransition(Event event) {

        return true;
    } 
}

package frc.robot;

import java.util.Optional;

import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Wristavator;
import frc.robot.subsystems.Climber.ClimberState;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Launcher.LauncherState;
import frc.robot.subsystems.Swerve.SwerveState;
import frc.robot.subsystems.Wristavator.WristavatorState;

public class RobotState {
 
    public static RobotState instance;

    /**
     * Collection of the Robot States, actions it can undertake in a match
     */
    public enum State {

        DRIVING,        //Default state
        INTAKING,       //When Intake Request triggers
        OUTTAKING,      //When Outtake Request triggers
        AIMING_SPEAKER,  
        AIMING_AMP,  
        LAUNCHING,      //When Aiming Complete triggers
        PATHING,        //when Pathing Request triggers
        CLIMB_LOCK,       //when Climb Request triggers
        CLIMB_EXTEND,
        CLIMB_RETRACT,
        IDLE            //Disabled state
    }

    /**
     * Collection of Robot Events, things that can physically be triggered in a match
     */
    public enum Event {

        INTAKE_REQUEST,  //triggered by aux button
        OUTTAKE_REQUEST, //triggered by aux button
        INTAKE_CANCEL,   //triggered when aux lets go
        HOLDER_PROX,     //triggered when note passes intake prox
        LAUNCH_SPEAKER_REQUEST,  //triggered by aux button
        LAUNCH_AMP_REQUEST,
        AIMING_COMPLETE, //triggered by launcher when aligned and at rpm
        LAUNCHER_SHOT,   //triggered by reading rpm drop and no "magazine" prox
        PATHING_REQUEST, //triggered by driver button
        CLIMB_LOCK_REQUEST,  //triggered by aux button
        CLIMB_RETRACT_REQUEST,
        CLIMB_EXTENDED_REQUEST,
        PATHING_COMPLETE,
        CLIMB_UNLOCK_REQUEST,
        CLIMB_CANCEL,
        
    }

    State currentState;
    Optional<State> desiredState;

    Optional<Event> mostRecentEvent;

    boolean validTransition;

    public RobotState() {
        currentState = State.IDLE;
        desiredState = Optional.of(State.IDLE);

        mostRecentEvent = Optional.empty();
    }

    public synchronized void setEvent(Event event) {
        mostRecentEvent = Optional.of(event);
        
    }

    public synchronized void periodic() {
        
        currentState = desiredState.get();
        
        if(mostRecentEvent.isPresent()) {
            switch (mostRecentEvent.get()) {
                case INTAKE_REQUEST:

                    //lower intake and run it

                    desiredState = Optional.of(State.INTAKING);
                    break;

                case OUTTAKE_REQUEST:

                    //lower intake and run reverse

                    desiredState = Optional.of(State.OUTTAKING);
                    break;
                
                case INTAKE_CANCEL:
                break;
                case HOLDER_PROX:

                    //stop running intake and stow
                    
                    desiredState = Optional.of(State.DRIVING);
                    break;

                case LAUNCH_SPEAKER_REQUEST:

                    //set limelight to speaker track
                    //spool up launcher and angler

                    desiredState = Optional.of(State.AIMING_SPEAKER);
                    break;

                case LAUNCH_AMP_REQUEST:

                    desiredState = Optional.of(State.AIMING_AMP);
                    break;

                case AIMING_COMPLETE:

                    //hold rpm and angle values
                    //give green light to load note into launcher
                    
                    desiredState = Optional.of(State.LAUNCHING);
                    break;

                case LAUNCHER_SHOT:

                    //bring launcher back to idle
                    
                    desiredState = Optional.of(State.DRIVING);
                    break;

                case PATHING_REQUEST:

                    //override driver controls for generated path
                    
                    desiredState = Optional.of(State.PATHING);
                    break;
                
                case CLIMB_RETRACT_REQUEST:

                    desiredState = Optional.of(State.CLIMB_RETRACT);
                    break;

                case CLIMB_EXTENDED_REQUEST:
                    desiredState = Optional.of(State.CLIMB_EXTEND);
                    break;
                
                case CLIMB_LOCK_REQUEST:
                    desiredState = Optional.of(State.CLIMB_LOCK);
                    break;
                
                case CLIMB_UNLOCK_REQUEST:
                    desiredState = Optional.of(State.DRIVING);
                    break;


                case CLIMB_CANCEL:
                    desiredState = Optional.of(State.DRIVING);
                    break;
            
                default:

                    desiredState = Optional.of(State.DRIVING);
                    break;
            }

            //reset the event
            mostRecentEvent = Optional.empty();
        }

        switch (currentState) {
            case DRIVING:     //Default state
            
                Swerve.getInstance().setState(SwerveState.DRIVING);
                Launcher.getInstance().setState(LauncherState.HOME);
                Intake.getInstance().setState(IntakeState.HOME);
                // Wristavator.getInstance().setState(WristavatorState.STOWED);
                Climber.getInstance().setState(ClimberState.HOME);
                break;

            case INTAKING:       //When Intake Request triggers
                
                Swerve.getInstance().setState(SwerveState.DRIVING);
                Launcher.getInstance().setState(LauncherState.SERIALIZE_NOTE);
                Intake.getInstance().setState(IntakeState.INTAKING);
                // Wristavator.getInstance().setState(WristavatorState.INTAKING);
                break;

            case OUTTAKING:     //When Outtake Request triggers

                Swerve.getInstance().setState(SwerveState.DRIVING);
                Launcher.getInstance().setState(LauncherState.HOME);
                Intake.getInstance().setState(IntakeState.OUTTAKING);
                // Wristavator.getInstance().setState(WristavatorState.INTAKING);
                break;

            case AIMING_SPEAKER: 
                
                Swerve.getInstance().setState(SwerveState.AIMING_SPEAKER);
                Launcher.getInstance().setState(LauncherState.AIMING_SPEAKER);
                Intake.getInstance().setState(IntakeState.HOME);
                // Wristavator.getInstance().setState(WristavatorState.LAUNCHING_SPEAKER);
                break;

            case AIMING_AMP: 
                
                Swerve.getInstance().setState(SwerveState.AIMING_AMP);
                Launcher.getInstance().setState(LauncherState.AIMING_AMP);
                Intake.getInstance().setState(IntakeState.HOME);
                // Wristavator.getInstance().setState(WristavatorState.LAUNCHING_AMP);
                break;

            case LAUNCHING:      //When Aiming Complete triggers
                
                Swerve.getInstance().setState(SwerveState.X_LOCKED);
                Launcher.getInstance().setState(LauncherState.LAUNCHING);
                Intake.getInstance().setState(IntakeState.HOME);
                break;
           

            case PATHING:       //when Pathing Request triggers

                Swerve.getInstance().setState(SwerveState.PATHING);
                Launcher.getInstance().setState(LauncherState.HOME);
                Intake.getInstance().setState(IntakeState.HOME);
                // Wristavator.getInstance().setState(WristavatorState.STOWED);
                break;

            case CLIMB_EXTEND:       //when Climb Request triggers
            Climber.getInstance().setState(ClimberState.EXTENDING);
                break;

            case CLIMB_LOCK:
            Climber.getInstance().setState(ClimberState.LOCKED);
                break;

            case CLIMB_RETRACT:
            Climber.getInstance().setState(ClimberState.RETRACTING);
                break;

            case IDLE:
            default:

                Swerve.getInstance().setState(SwerveState.HOME); 
                Launcher.getInstance().setState(LauncherState.HOME);
                Intake.getInstance().setState(IntakeState.HOME);
                // Wristavator.getInstance().setState(WristavatorState.HOME);
                Climber.getInstance().setState(ClimberState.HOME);
                break;
        }

        Intake.getInstance().reportData();
        Launcher.getInstance().reportData();
        // Wristavator.getInstance().reportData();
    }



    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        
        return instance;
    }
}

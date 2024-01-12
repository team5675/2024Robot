package frc.robot;

import java.util.Optional;

import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Launcher;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Intake.IntakeState;
import frc.robot.subsystems.Launcher.LauncherState;
import frc.robot.subsystems.Swerve.SwerveState;

public class RobotState {
 
    public static RobotState instance;

    /**
     * Collection of the Robot States, actions it can undertake in a match
     */
    public enum State {

        DRIVING,        //Default state
        INTAKING,       //When Intake Request triggers
        OUTTAKING,      //When Outtake Request triggers
        AIMING_SPEAKER_LAZY,    //When Launch Request triggers
        AIMING_SPEAKER_REAL,  
        AIMING_AMP_LAZY,
        AIMING_AMP_REAL,  
        LAUNCHING,      //When Aiming Complete triggers
        PATHING,        //when Pathing Request triggers
        CLIMBING,       //when Climb Request triggers
        IDLE            //Disabled state
    }

    /**
     * Collection of Robot Events, things that can physically be triggered in a match
     */
    public enum Event {

        INTAKE_REQUEST,  //triggered by aux button
        OUTTAKE_REQUEST, //triggered by aux button
        INTAKE_CANCEL,   //triggered when aux lets go
        INTAKE_PROX,     //triggered when note passes intake prox
        LAUNCHER_PROX,   //triggered when note passes "magazine" prox
        LAUNCH_SPEAKER_REQUEST,  //triggered by aux button
        LAUNCH_AMP_REQUEST,
        AIMING_COMPLETE, //triggered by launcher when aligned and at rpm
        LAUNCHER_SHOT,   //triggered by reading rpm drop and no "magazine" prox
        PATHING_REQUEST, //triggered by driver button
        CLIMB_REQUEST,  //triggered by aux button
        PATHING_COMPLETE,
        CLIMB_COMPLETE;
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
                case INTAKE_PROX:

                    //stop running intake and stow
                    
                    desiredState = Optional.of(State.DRIVING);
                    break;

                case LAUNCHER_PROX:

                    //stop running serializer??
                    
                    desiredState = Optional.of(State.AIMING_SPEAKER_LAZY);
                    break;

                case LAUNCH_SPEAKER_REQUEST:

                    //set limelight to speaker track
                    //spool up launcher and angler

                    desiredState = Optional.of(State.AIMING_SPEAKER_REAL);
                    break;

                case LAUNCH_AMP_REQUEST:

                    desiredState = Optional.of(State.AIMING_AMP_REAL);
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
                break;

            case INTAKING:       //When Intake Request triggers

                Swerve.getInstance().setState(SwerveState.DRIVING);
                Launcher.getInstance().setState(LauncherState.HOME);
                Intake.getInstance().setState(IntakeState.INTAKING);
                break;

            case OUTTAKING:     //When Outtake Request triggers

                Swerve.getInstance().setState(SwerveState.DRIVING);
                Launcher.getInstance().setState(LauncherState.HOME);
                Intake.getInstance().setState(IntakeState.OUTTAKING);
                break;

            case AIMING_SPEAKER_LAZY:    //When Launch Request triggers

                Swerve.getInstance().setState(SwerveState.DRIVING);
                Launcher.getInstance().setState(LauncherState.AIMING_SPEAKER_LAZY);
                Intake.getInstance().setState(IntakeState.HOME);
                break;

            case AIMING_SPEAKER_REAL: 

                Swerve.getInstance().setState(SwerveState.DRIVING);
                Launcher.getInstance().setState(LauncherState.AIMING_SPEAKER_REAL);
                Intake.getInstance().setState(IntakeState.HOME);
                break;

            case AIMING_AMP_LAZY:

                Swerve.getInstance().setState(SwerveState.DRIVING);
                Launcher.getInstance().setState(LauncherState.AIMING_AMP);
                Intake.getInstance().setState(IntakeState.HOME);
                break;

            case AIMING_AMP_REAL: 
            
                Swerve.getInstance().setState(SwerveState.DRIVING);
                Launcher.getInstance().setState(LauncherState.AIMING_AMP);
                Intake.getInstance().setState(IntakeState.HOME);
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
                break;

            case CLIMBING:       //when Climb Request triggers

                Swerve.getInstance().setState(SwerveState.DRIVING);
                Launcher.getInstance().setState(LauncherState.HOME);
                Intake.getInstance().setState(IntakeState.HOME);
                break;

            case IDLE:
            default:

                Swerve.getInstance().setState(SwerveState.HOME); 
                Launcher.getInstance().setState(LauncherState.HOME);
                Intake.getInstance().setState(IntakeState.HOME);
                break;
        }

        Intake.getInstance().reportData();
        Launcher.getInstance().reportData();
    }



    public static RobotState getInstance() {
        if (instance == null)
            instance = new RobotState();
        
        return instance;
    }
}

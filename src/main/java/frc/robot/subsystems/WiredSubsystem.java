package frc.robot.subsystems;

public interface WiredSubsystem {

    public interface InnerWiredSubsystemState {};

    public InnerWiredSubsystemState getState();

    public void reportData();

    
}

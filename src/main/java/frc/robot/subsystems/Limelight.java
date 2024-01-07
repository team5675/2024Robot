package frc.robot.subsystems;

public class Limelight {
    

    public static Limelight instance;


    public static Limelight getInstance() {
        if(instance == null)
            instance = new Limelight();

        return instance;
    }
}

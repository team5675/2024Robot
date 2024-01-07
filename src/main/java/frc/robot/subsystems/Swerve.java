package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Swerve {
    
    public static Swerve instance;

    public Trigger getPathComplete() {

        BooleanSupplier test = new BooleanSupplier() {
            int i = 0;

            @Override
            public boolean getAsBoolean() {
                //get when path is done
                return i > 0;
            }
        };

        return new Trigger(test);
    }


    public static Swerve getInstance() {
        if (instance == null) 
            instance = new Swerve();
        
        return instance;
    }
}

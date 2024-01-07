package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake {
    
    public static Intake instance;


    public Trigger getIntakeProx() {

        BooleanSupplier test = new BooleanSupplier() {
            int i = 0;

            @Override
            public boolean getAsBoolean() {
                return i > 0;
            }
        };

        return new Trigger(test);
    }


    public static Intake getInstance() {
        if (instance == null) 
            instance = new Intake();

        return instance;
    }
}

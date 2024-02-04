package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Climber {
    
    static Climber instance;

    public Trigger getClimbComplete() {

        BooleanSupplier test = new BooleanSupplier() {
            int i = 0;

            @Override
            public boolean getAsBoolean() {
                //get complete climb conditions
                return i > 0;
            }
        };

        return new Trigger(test);
    }


    public static Climber getInstance() {
        if(instance == null)
            instance = new Climber();

        return instance;
    }
}

package frc.robot.subsystems;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Launcher {
    
    public static Launcher instance;
    
    public Trigger getLauncherProx() {

        BooleanSupplier test = new BooleanSupplier() {
            int i = 0;

            @Override
            public boolean getAsBoolean() {
                //get when prox has been triggered
                return i > 0;
            }
        };

        return new Trigger(test);
    }

    public Trigger getAimingComplete() {

        BooleanSupplier test = new BooleanSupplier() {
            int i = 0;

            @Override
            public boolean getAsBoolean() {
                //get when rpm and angle are good
                return i > 0;
            }
        };

        return new Trigger(test);
    }

    public Trigger getLauncherShot() {

        BooleanSupplier test = new BooleanSupplier() {
            int i = 0;

            @Override
            public boolean getAsBoolean() {
                //get when launcher shot
                return i > 0;
            }
        };

        return new Trigger(test);
    }


    public static Launcher getInstance() {
        if (instance == null) 
            instance = new Launcher();

        return instance;
    }
}

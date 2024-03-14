// Copyright (c) 2023 FRC 5675
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file at
// the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public final class LEDs {

  public static LEDs instance;
  private Spark blinkinLED; //Maybe Static?

  public void createBlinkin() {
    blinkinLED = new Spark(1);
  }

  public void setRainbow() {
    blinkinLED.set(-0.99); //Rainbow
  }

  public void setOrange() {
    blinkinLED.set(0.65); //Orange
  }

  public void setAllianceColor() {

    var alliance = DriverStation.getAlliance();
    if(alliance.isPresent()){
      if(alliance.get() == DriverStation.Alliance.Blue){
        blinkinLED.set(0.87); //Blue
        } else {
          blinkinLED.set(0.59); //0.61 for normal red
        }
      }
  }
  public static LEDs getInstance() {
    if (instance == null) 
        instance = new LEDs();

    return instance;
}
}

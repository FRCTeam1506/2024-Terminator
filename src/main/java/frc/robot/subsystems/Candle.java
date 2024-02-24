// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;


public class Candle extends SubsystemBase {
  /** Creates a new Candle. */
  CANdle candle = new CANdle(Constants.CandleSubsystem.CANDLE_ID);

  public Candle() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 1; // dim the LEDs to half brightness
    candle.configAllSettings(config);
  }

  public void yellow(){
    // stopGSA();
    // color = "yellow";
    // candle.setLEDs(rY, gY, bY); // set the CANdle LEDs to white
    // Constants.CandleSubsystem.cone = true;

  }

  public void gsa(){
    RainbowAnimation rainbowAnim = new RainbowAnimation(1, 0.5, 64);
    candle.animate(rainbowAnim);
  }

  public void stopGSA(){
      candle.animate(null);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

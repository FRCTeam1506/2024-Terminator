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
import com.ctre.phoenix.led.StrobeAnimation;


public class Candle extends SubsystemBase {
  /** Creates a new Candle. */
  CANdle candle = new CANdle(Constants.CandleSubsystem.CANDLE_ID);

  int[] orange = {0, 0, 255};
  int[] green = {0, 191, 0};
  int[] red = {191, 0, 0};


  public Candle() {
    CANdleConfiguration config = new CANdleConfiguration();
    config.stripType = LEDStripType.RGB; // set the strip type to RGB
    config.brightnessScalar = 1; // dim the LEDs to half brightness
    candle.configAllSettings(config);
  }

  public void orange(){
    stopGSA();
    candle.setLEDs(orange[0], orange[1], orange[2]);
    Constants.CandleSubsystem.note = true;
    
  }

  public void strobeOrange(){
    stopGSA();
    StrobeAnimation strobe = new StrobeAnimation(orange[0], orange[1], orange[2]);
    candle.animate(strobe);
  }

  public void green(){
    stopGSA();
    candle.setLEDs(green[0], green[1], green[2]);
  }

  public void strobeGreen(){
    stopGSA();
    StrobeAnimation strobe = new StrobeAnimation(green[0], green[1], green[2]);
    candle.animate(strobe);
  }

  public void red(){
    stopGSA();
    candle.setLEDs(red[0], red[1], red[2]);
  }

  public void strobeRed(){
    stopGSA();
    StrobeAnimation strobe = new StrobeAnimation(red[0], red[1], red[2]);
    candle.animate(strobe);
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
    if(!Constants.IntakeSubsystem.irNine.get()){
      if(Vision.shooterID == 4 || Vision.shooterID == 7){
        green();
      }else{
        orange();
      }
    }
    else{
      red();
    }

  }
}

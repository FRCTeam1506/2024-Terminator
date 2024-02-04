// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.math.Conversions;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private TalonFX motor1 = new TalonFX(Constants.ShooterSubsystem.TopID);
  private TalonFX motor2 = new TalonFX(Constants.ShooterSubsystem.BottomID);
  private double ratio = 18/24;
  double maxVelocity = 1;
  double midVelocity = .5;
  double minVelocity = .3;

  public ShooterSubsystem() {

    MotorOutputConfigs output = new MotorOutputConfigs();
    output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput = output;

    motor1.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);

  }

  public void shootMax(){
    motor1.set(maxVelocity);
    motor2.set(maxVelocity);
    System.out.println("Shoot Max");
  }

  public void shootMid(){
    motor1.set(midVelocity);
    motor2.set(midVelocity);
  }

  public void shootMin(){
    motor1.set(minVelocity);
    motor2.set(minVelocity);
  }

  double increment = 0.5;
  public void ShootIncrement(){
    motor1.set(increment);
    motor2.set(increment);
  }

  public void incrementUp(){
    increment+=0.1;
  }

  public void incrementDown(){
    increment-=0.1;
  }

  public void shootMaxRPM(){
    // motor1.set(TalonFXControlMode.Velocity, Conversions.RPMToFalcon(5000, ratio));
    // motor2.set(TalonFXControlMode.Velocity, Conversions.RPMToFalcon(5000, ratio));

    // motor1.set(11377);
    // motor2.set(11377);

    VelocityDutyCycle request = new VelocityDutyCycle(3000);
    motor1.setControl(request);
    motor2.setControl(request);

  }

  public void shootMinRPM(){
    // motor1.set(Conversions.RPMToFalcon(3000, ratio));
    // motor2.set(Conversions.RPMToFalcon(3000, ratio));
    VelocityDutyCycle request = new VelocityDutyCycle(3000);
    motor1.setControl(request);
    motor2.setControl(request);
  }


  public void shootStop(){
    motor1.set(0);
    motor2.set(0);
  }

  public double getIncrement(){
    return increment;
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}

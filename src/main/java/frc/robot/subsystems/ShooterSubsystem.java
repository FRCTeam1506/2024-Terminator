// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import javax.swing.text.Position;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DifferentialMotionMagicVoltage;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Position;
import com.ctre.phoenix6.controls.compound.Diff_PositionDutyCycle_Position;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.math.Conversions;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private TalonFX motor1 = new TalonFX(Constants.ShooterSubsystem.TopID);
  private TalonFX motor2 = new TalonFX(Constants.ShooterSubsystem.BottomID);
  private TalonFX angler = new TalonFX(Constants.ShooterSubsystem.AnglerID);
  Vision vision;
  private double ratio = 18/24;
  double maxVelocity = 1;
  double midVelocity = .5;
  double minVelocity = .3;
  MotionMagicVoltage m_mmReq;

  public ShooterSubsystem(Vision vision) {

    this.vision = vision;

    MotorOutputConfigs output = new MotorOutputConfigs();
    output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput = output;

    motor1.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);
    angler.getConfigurator().apply(config);

    angler.setNeutralMode(NeutralModeValue.Brake);
    angler.setInverted(false);
    motor1.setInverted(false);

    // set Motion Magic settings
    /* Configure current limits */
    m_mmReq = new MotionMagicVoltage(0);
    MotionMagicConfigs mm = config.MotionMagic;
    mm.MotionMagicCruiseVelocity = 0.5; // 5 rotations per second cruise
    mm.MotionMagicAcceleration = 5; // Take approximately 0.5 seconds to reach max vel OG 10
    // Take approximately 0.2 seconds to reach max accel 
    mm.MotionMagicJerk = 50;

    Slot0Configs slot0 = config.Slot0;
    slot0.kP = 1;//60 //2
    slot0.kI = .5;//0
    slot0.kD = 0.1;
    slot0.kV = 0.12;
    slot0.kS = 0.25; // Approximately 0.25V to get the mechanism moving

    FeedbackConfigs fdb = config.Feedback;
    fdb.SensorToMechanismRatio = 48.89; //12.8 //48.89

    StatusCode status = StatusCode.StatusCodeNotInitialized;
    for(int i = 0; i < 5; ++i) {
      status = angler.getConfigurator().apply(config);
      if (status.isOK()) break;
    }
    if (!status.isOK()) {
      System.out.println("Could not configure device. Error: " + status.toString());
    }
    angler.setInverted(false);


  }

  public void shoot(){
    
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
    increment+=0.05;
  }

  public void incrementDown(){
    increment-=0.05;
  }

  public void shootMaxRPM(){
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

  public double getDistance(){
    return 6*Math.sin(vision.y*Math.PI/180);
  }

  public double getNecessaryPower(){
    double x = 0;
    return x;
  }

  public void anglerUp(){
    angler.set(Constants.ShooterSubsystem.anglerDefaultSpeed);
  }

  public void anglerDown(){
    angler.set(-Constants.ShooterSubsystem.anglerDefaultSpeed);
  }

  public void anglerStop(){
    angler.stopMotor();
    angler.set(0);
  }

  public void setAngler(double setpoint){
    // MotionMagicDutyCycle set = new MotionMagicDutyCycle(setpoint);
    // angler.setControl(set);
    
    // Diff_DutyCycleOut_Position pos = new Diff_DutyCycleOut_Position(new DutyCycleOut(0.5),new PositionDutyCycle(setpoint));
    // angler.setControl(pos);

    // angler.setPosition(2);
    System.out.println("ANGLER!!!!!");

    DifferentialMotionMagicVoltage pos = new DifferentialMotionMagicVoltage(setpoint, setpoint);
    angler.setControl(pos.withTargetPosition(setpoint));
  }

  boolean finished = false;
  public void setAnglerOld(double setpoint){
    if(getAngle() > setpoint + 0.1 && getAngle() < setpoint - 0.1 || finished){
      anglerStop();
      finished = true;
    }
    else if(getAngle() > setpoint){
      anglerDown();
      finished = false;
    }
    else if (getAngle() < setpoint){
      anglerUp();
      finished = false;
    }
  }

  public void finishedReset(){
    finished = false;
  }

  public double getAngle(){
    return angler.getRotorPosition().getValueAsDouble();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    
  }
}

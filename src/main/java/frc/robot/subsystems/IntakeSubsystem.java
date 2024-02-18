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

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
// import frc.lib.math.Conversions;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private TalonFX motor = new TalonFX(Constants.IntakeSubsystem.MOTOR_ID);
  private TalonFX indexer = new TalonFX(Constants.IntakeSubsystem.INDEXER_ID);
  private DigitalInput irNine = new DigitalInput(7);
  private DigitalInput irEight = new DigitalInput(6);

  ShuffleboardTab tab = Shuffleboard.getTab("Vision");

  Timer timer = new Timer();


  public IntakeSubsystem() {

    MotorOutputConfigs output = new MotorOutputConfigs();
    output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput = output;

    motor.getConfigurator().apply(config);
    indexer.getConfigurator().apply(config);

  }

  public void justIntake(){
    motor.set(Constants.IntakeSubsystem.DEFAULT_INTAKE_SPEED);
    // indexer.set(maxVelocity);
    System.out.println("Intake");

  }

  public void intake() {

    //by motor torque
    // if(indexer.getTorqueCurrent().getValueAsDouble() > 23 || Constants.IntakeSubsystem.ring){
    //   Constants.IntakeSubsystem.ring = true;
    //   // Timer.delay(.1);
    //   stopIntake();
    // }

    if(!irNine.get()){
      Constants.IntakeSubsystem.ring = true;
      // Timer.delay(.1);
      stopIntake();
    }
    else{
      motor.set(Constants.IntakeSubsystem.DEFAULT_INTAKE_SPEED);
      indexer.set(Constants.IntakeSubsystem.DEFAULT_INDEXER_SPEED);
    }
  }


  public void outtake(){
    motor.set(-Constants.IntakeSubsystem.DEFAULT_INTAKE_SPEED);
    indexer.set(-Constants.IntakeSubsystem.DEFAULT_INDEXER_SPEED);
    // indexer.set(maxVelocity);
    System.out.println("Outtake");
  }

  public void intakeRPM(){
    VelocityDutyCycle request = new VelocityDutyCycle(3000);
    motor.setControl(request);
  }


  public void stopIntake(){
    motor.set(0);
    indexer.set(0);
  }

  public void stopIntakeWithReset(){
    motor.set(0);
    indexer.set(0);
    resetRing();
  }

  public void runIndexerShoot(){
    indexer.set(.5);
  }


  public void stopIndexer(){
    indexer.set(0);
  }

  public void stop(){
    motor.set(0);
    indexer.set(0);
  }

  public double getTorqueCurrent(){
    return indexer.getTorqueCurrent().getValueAsDouble();
  }

  public void resetRing(){
    Constants.IntakeSubsystem.ring = false;
  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // System.out.println("infra eight: " + irEight.get());    
    // System.out.println("infra nine: " + irNine.get());

    SmartDashboard.putBoolean("ir 8", irEight.get());
    SmartDashboard.putBoolean("ir 9", irNine.get());
    
    
    if(Constants.IntakeSubsystem.ring){
      // tab.ad
    }


  }
}

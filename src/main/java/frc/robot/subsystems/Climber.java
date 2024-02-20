// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Climber extends SubsystemBase {
  /** Creates a new Climber. */
  //directions as when looking at the shooter
  TalonFX left = new TalonFX(Constants.ClimberSubsystem.LEFT_ID);
  TalonFX right = new TalonFX(Constants.ClimberSubsystem.RIGHT_ID);
  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  public Climber() {
    TalonFXConfiguration talonFXConfigs = new TalonFXConfiguration();
    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    
    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 4.8;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;
    slot0Configs.GravityType = GravityTypeValue.Elevator_Static;

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 10; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 8; // 160 rps/s acceleration (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)
    

    left.getConfigurator().apply(talonFXConfigs, 0.050);
    right.getConfigurator().apply(talonFXConfigs, 0.050);
    left.setPosition(0);
    right.setPosition(0);
    right.setInverted(true);

  }

  public void up(){
    left.set(Constants.ClimberSubsystem.DEFAULT_SPEED);
    right.set(Constants.ClimberSubsystem.DEFAULT_SPEED);
  }

  public void down(){
    left.set(-Constants.ClimberSubsystem.DEFAULT_SPEED);
    right.set(-Constants.ClimberSubsystem.DEFAULT_SPEED);
  }

  public void climb(){
    m_motmag.Slot = 0;
    left.setControl(m_motmag.withPosition(Constants.ClimberSubsystem.CLIMB_POSITION));
    right.setControl(m_motmag.withPosition(Constants.ClimberSubsystem.CLIMB_POSITION));
  }

  public void stop(){
    left.stopMotor();
    right.stopMotor();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Angler extends SubsystemBase {
  /** Creates a new Angler. */
  TalonFX motor = new TalonFX(Constants.ShooterSubsystem.AnglerID);
  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  public Angler() {
    // robot init
    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    // set slot 0 gains
    var slot0Configs = talonFXConfigs.Slot0;
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // PID runs on position
    slot0Configs.kP = 4.8;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;

    // set Motion Magic settings
    var motionMagicConfigs = talonFXConfigs.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = 80; // 80 rps cruise velocity
    motionMagicConfigs.MotionMagicAcceleration = 160; // 160 rps/s acceleration (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 1600; // 1600 rps/s^2 jerk (0.1 seconds)

    motor.getConfigurator().apply(talonFXConfigs, 0.050);
    motor.setPosition(0);

    // SmartDashboard.putNumber("angler set", getVisionPosition());

  }

  public void setPosition(){
    m_motmag.Slot = 0;
    double dist = Vision.shotdist;
    double pos = 0.228874*Math.pow(dist, 2) - 2.72467*dist + 8.70407; //desmos eq, check screenshots 2/21/2024 +++
    motor.setControl(m_motmag.withPosition(pos));
  }

  public void setPositionManual(double position){
    m_motmag.Slot = 0;
    motor.setControl(m_motmag.withPosition(position));
  }

  public void setPositionIncrement(){
    m_motmag.Slot = 0;
    motor.setControl(m_motmag.withPosition(new ShooterSubsystem().getIncrement()));
  }

  public void anglerUp(){
    motor.set(0.3);
  }

  public void anglerDown(){
    motor.set(-0.3);
  }

  public void ampPosition(){
    m_motmag.Slot = 0;
    motor.setControl(m_motmag.withPosition(4.8));
  }

  public void stopAngler(){
    motor.set(0);
    motor.stopMotor();
  }

  public void anglerZero(){
    motor.setPosition(0);
  }

  public double getPos(){
    return motor.getPosition().getValueAsDouble();
  }

  public double getVisionPosition(){
    // double number = Vision.z;
    return 0;
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
        SmartDashboard.putNumber("angler set", getVisionPosition());

  }
}

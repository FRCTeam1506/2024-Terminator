// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

public class Angler extends SubsystemBase {
  /** Creates a new Angler. */
  TalonFX motor = new TalonFX(Constants.ShooterSubsystem.AnglerID);
  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  public GenericEntry dashangler = Shuffleboard.getTab("Robot").add("setangler", 6)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 6))
    .getEntry();


  public Angler() {
    // robot init
    var talonFXConfigs = new TalonFXConfiguration();

    talonFXConfigs.MotorOutput.NeutralMode = NeutralModeValue.Brake;

    talonFXConfigs.CurrentLimits.StatorCurrentLimitEnable = true;
    talonFXConfigs.CurrentLimits.StatorCurrentLimit = 120; //maybe change this

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

    double xshot = Math.abs(LimelightHelpers.getBotPose_TargetSpace("limelight")[0]);
    // xshot = 1;
    double zshot = Math.abs(LimelightHelpers.getBotPose_TargetSpace("limelight")[2]);
    // zshot = 1;
    double dist = Math.sqrt(Math.pow(Math.abs(xshot), 2) + Math.pow(Math.abs(zshot), 2)); //pythagorean theorem


    if(Vision.target != 0){
      /*
      if(DriverStation.getAlliance().get() == Alliance.Blue){
        double pos = 0.228874*Math.pow(dist, 2) - 2.72467*dist + 8.70407; //desmos eq, check screenshots 2/21/2024 +++
        //double pos = 0.3271*Math.pow(dist, 2) - 3.73081*dist + 11.2833; //for week 0
        motor.setControl(m_motmag.withPosition(pos));
      }
      else if(DriverStation.getAlliance().get() == Alliance.Red){
        //double pos = 0.228874*Math.pow(dist, 2) - 2.72467*dist + 8.70407; //desmos eq, check screenshots 2/21/2024 +++
        double pos = 0.3271*Math.pow(dist, 2) - 3.73081*dist + 11.2833; //for week 0
        motor.setControl(m_motmag.withPosition(pos));
      }
      */
      //double pos = 0.3271*Math.pow(dist, 2) - 3.73081*dist + 11.2833; //for week 0 and kettering week 1
/*
      // KETTERING WEEK ONE
      double pos = 0.228874*Math.pow(dist, 2) - 2.72467*dist + 8.70407; //desmos eq, check screenshots 2/21/2024 +++

      if(DriverStation.getAlliance().get() == Alliance.Blue){
        motor.setControl(m_motmag.withPosition(pos + 0.05)); //we are shooting low on blue alliance //0.2 too high //+0.05 for k1
      }
      else{
        motor.setControl(m_motmag.withPosition(pos + 0.07)); //shooting high on red //+0.07 for k1
      }
*/      

       double pos = 0.272199*Math.pow(dist, 2) - 2.74293*dist + 8.213; //desmos eq, check screenshots 2/21/2024 +++
       motor.setControl(m_motmag.withPosition(pos));
    }
  }

  public void setPositionManual(double position){
    m_motmag.Slot = 0;
    motor.setControl(m_motmag.withPosition(position));
  }

  public void goVertical(){
    m_motmag.Slot = 0;
    double vertPos = 12.5;
    motor.setControl(m_motmag.withPosition(vertPos));
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

  public double getShuffleboardInput(){
    return dashangler.getDouble(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putNumber("angler set", getVisionPosition());
  }
}

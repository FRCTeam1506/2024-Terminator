// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem5 extends SubsystemBase {
  
  private TalonFX motor1 = new TalonFX(Constants.ShooterSubsystem.TopID);
  private TalonFX motor2 = new TalonFX(Constants.ShooterSubsystem.BottomID);
  private TalonFX angler = new TalonFX(Constants.ShooterSubsystem.AnglerID);
  
  Vision vision;
  private double ratio = 18/24;
  double maxVelocity = 1;
  double midVelocity = .5;
  double minVelocity = .3;

  double encoderCount = angler.getSelectedSensorPosition();
  double startingEncoderCount = encoderCount;
  double speed = 0.3;
  private static final double kP = 0.61; // 0.84
  private static final double kI = 0.00025;
  private static final double kD = 0;
  private static final double kF = 0.4;  // 0.4

  private static final double kVelocity = 40_000.0;       // 62_000.0
  private static final double kAcceleration = 30_000.0;   // 44_000.0

  private static final double MIN_POSITION = -1_500.0;
  private static final double MAX_POSITION = 230_000.0; // 200_000


  public ShooterSubsystem5(Vision vision) {

    this.vision = vision;

    motor1.setInverted(TalonFXInvertType.Clockwise);
    motor2.setInverted(TalonFXInvertType.CounterClockwise);
    angler.setInverted(TalonFXInvertType.CounterClockwise);

    motor1.setNeutralMode(NeutralMode.Brake);
    motor2.setNeutralMode(NeutralMode.Brake);
    angler.setNeutralMode(NeutralMode.Brake);

    angler.config_kP(0, kP);
    angler.config_kI(0, kI);
    angler.config_kD(0, kD);
    angler.config_kF(0, kF);

    angler.configMotionCruiseVelocity(kVelocity);
    angler.configMotionAcceleration(kAcceleration);

    //current limit burn out the motor at states
    SupplyCurrentLimitConfiguration driveSupplyLimit = new SupplyCurrentLimitConfiguration(
        Constants.ShooterSubsystem.DRIVE_ENABLE_CURRENT_LIMIT, 
        Constants.ShooterSubsystem.DRIVE_CONTINUOUS_CL, 
        Constants.ShooterSubsystem.DRIVE_PEAK_CL, 
        Constants.ShooterSubsystem.DRIVE_PEAK_CURRENT_DURATION);

    angler.configSupplyCurrentLimit(driveSupplyLimit);
    motor1.configSupplyCurrentLimit(driveSupplyLimit);
    motor2.configSupplyCurrentLimit(driveSupplyLimit);



  }

  public void shoot(){
    
  }

  public void shootMax(){
    motor1.set(TalonFXControlMode.PercentOutput, maxVelocity);
    motor2.set(TalonFXControlMode.PercentOutput, maxVelocity);
    System.out.println("Shoot Max");
  }

  public void shootMid(){
    motor1.set(TalonFXControlMode.PercentOutput, midVelocity);
    motor2.set(TalonFXControlMode.PercentOutput, midVelocity);
  }


  public void shootRPM(double RPM){
    // VelocityDutyCycle request = new VelocityDutyCycle(3000);
    motor1.set(TalonFXControlMode.Velocity, RPM);
    motor2.set(TalonFXControlMode.Velocity, RPM);
  }


  public void shootStop(){
    motor1.set(TalonFXControlMode.PercentOutput, 0);
    motor2.set(TalonFXControlMode.PercentOutput, 0);
  }

  public double getDistance(){
    return 6*Math.sin(vision.y*Math.PI/180);
  }

  public double getNecessaryPower(){
    double x = 0;
    return x;
  }

  public void anglerUp(){
    angler.set(TalonFXControlMode.PercentOutput, Constants.ShooterSubsystem.anglerDefaultSpeed);
  }

  public void anglerDown(){
    angler.set(TalonFXControlMode.PercentOutput, -Constants.ShooterSubsystem.anglerDefaultSpeed);
  }

  public void anglerStop(){
    angler.set(TalonFXControlMode.PercentOutput, 0);
  }

  public void setAngler(double setpoint){
    // MotionMagicDutyCycle set = new MotionMagicDutyCycle(setpoint);
    // angler.setControl(set);
    
    // Diff_DutyCycleOut_Position pos = new Diff_DutyCycleOut_Position(new DutyCycleOut(0.5),new PositionDutyCycle(setpoint));
    // angler.setControl(pos);

    // angler.setPosition(2);
    System.out.println("ANGLER!!!!!");

    // DifferentialMotionMagicVoltage pos = new DifferentialMotionMagicVoltage(setpoint, setpoint);
    // angler.setControl(pos.withTargetPosition(setpoint));

    angler.set(TalonFXControlMode.MotionMagic, setpoint);
  }

  public double getAngle(){
    return angler.getSelectedSensorPosition();
  }

  public void resetMotors () {
    angler.setSelectedSensorPosition(0.0);
    System.out.println("reset" + angler.getSelectedSensorPosition());
}


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    encoderCount = angler.getSelectedSensorPosition();
  }
}

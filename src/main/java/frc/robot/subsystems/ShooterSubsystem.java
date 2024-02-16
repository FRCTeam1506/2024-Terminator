// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


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
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.controls.compound.Diff_DutyCycleOut_Position;
import com.ctre.phoenix6.controls.compound.Diff_PositionDutyCycle_Position;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);

  double encoderCount = angler.getPosition().getValue();
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




          // class member variable
    final PositionVoltage m_position = new PositionVoltage(.4); //.6 good for zone line  5.5 for subwoofer(50*)
    //^^^this was making the shooter rise. 2.5 good for stage leg
    // Trapezoid profile with max velocity 80 rps, max accel 160 rps/s
    final TrapezoidProfile m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(10, 8)
    );
    // Final target of 200 rot, 0 rps
    TrapezoidProfile.State m_goal = new TrapezoidProfile.State(.4, 0);
    TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();



  public ShooterSubsystem(Vision vision) {

    this.vision = vision;

    MotorOutputConfigs output = new MotorOutputConfigs();
    output.Inverted = InvertedValue.Clockwise_Positive;
    output.NeutralMode = NeutralModeValue.Brake;

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput = output;
    config.MotionMagic.MotionMagicAcceleration = kVelocity;
    config.MotionMagic.MotionMagicAcceleration = kAcceleration;

    motor1.getConfigurator().apply(config);
    motor2.getConfigurator().apply(config);
    angler.getConfigurator().apply(config);

    angler.setNeutralMode(NeutralModeValue.Brake);
    angler.setInverted(false);
    motor1.setInverted(false);

    // Slot0Configs vegas = new Slot0Configs();

    // vegas.kP = kP;
    // vegas.kI = kI;
    // vegas.kD = kD;
    // vegas.kS = 0.24; // add 0.24 V to overcome friction
    // vegas.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    // vegas.GravityType = GravityTypeValue.Arm_Cosine;

    // angler.getConfigurator().apply(vegas);

    angler.setPosition(0);

    configAngler();
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
    angler.set(0);
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
    // System.out.println("ANGLER!!!!!");

    // DifferentialMotionMagicVoltage pos = new DifferentialMotionMagicVoltage(setpoint, setpoint);
    // angler.setControl(pos.withTargetPosition(setpoint));

    m_motmag.Slot = 0;
    angler.setControl(m_motmag.withPosition(setpoint));
  }

  public double getAngle(){
    return angler.getRotorPosition().getValueAsDouble();
  }

  public double getAvgShooterSpeed(){
    return (motor1.getVelocity().getValueAsDouble() + motor2.getVelocity().getValueAsDouble())/2;
  }

  public void configAngler(){
    // robot init, set slot 0 gains
    var slot0Configs = new Slot0Configs();
    slot0Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot0Configs.kV = 0.12; // apply 12 V for a target velocity of 100 rps
    slot0Configs.kP = 4.8;
    slot0Configs.kI = 0;
    slot0Configs.kD = 0.1;
    angler.getConfigurator().apply(slot0Configs, 0.050);

  }

  public void setAnglerNew(){
    // periodic, update the profile setpoint for 20 ms loop time
    m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);
    // apply the setpoint to the control request
    m_position.Position = m_setpoint.position;
    m_position.Velocity = m_setpoint.velocity;
    angler.setControl(m_position);

  }

  public void shootRing(){
    setAnglerNew();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    encoderCount = angler.getPosition().getValue();
  }
}
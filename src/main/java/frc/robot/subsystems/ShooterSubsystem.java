// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /** Creates a new ShooterSubsystem. */

  private TalonFX motor1 = new TalonFX(Constants.ShooterSubsystem.TopID);
  private TalonFX motor2 = new TalonFX(Constants.ShooterSubsystem.BottomID);
  Vision vision;
  double maxVelocity = 1;
  double midVelocity = .5;
  double minVelocity = .3;

  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  double[] array = new double[30];

  double speed = 0.3;

  private static final double kVelocity = 40_000.0;       // 62_000.0
  private static final double kAcceleration = 30_000.0;   // 44_000.0

  private static final double MIN_POSITION = -1_500.0;
  private static final double MAX_POSITION = 230_000.0; // 200_000

  /* 
  // IMPORTANT ANGLER CODE
  final PositionVoltage m_position = new PositionVoltage(.4); //.6 good for zone line  5.5 for subwoofer(50*)
  //^^^this was making the shooter rise. 2.5 good for stage leg
  final TrapezoidProfile m_profile = new TrapezoidProfile(
    new TrapezoidProfile.Constraints(10, 8)
  );
*/

    // class member variable
    final PositionVoltage m_position = new PositionVoltage(0.5); //.6 good for zone line  5.5 for subwoofer(50*)
    //^^^this was making the shooter rise. 2.5 good for stage leg
    // Trapezoid profile with max velocity 80 rps, max accel 160 rps/s
    final TrapezoidProfile m_profile = new TrapezoidProfile(
      new TrapezoidProfile.Constraints(10, 8)
    );

    // Final target of 200 rot, 0 rps
    TrapezoidProfile.State m_goal;// = new TrapezoidProfile.State(increment, 0);
    TrapezoidProfile.State m_setpoint;// = new TrapezoidProfile.State();
  double increment = 0.6;

      
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

    motor1.setInverted(false);

    // m_goal = new TrapezoidProfile.State(5, 0);
    // m_setpoint = new TrapezoidProfile.State();



  }

  public void shoot(){
    
  }

  public void anglerSetpoint(){
    // Final target of 200 rot, 0 rps
    m_goal = new TrapezoidProfile.State(5, 0);
    m_setpoint = new TrapezoidProfile.State();

  }

  public void setAnglerNew(){

    /* 
    //.6 good for zone line  5.5 for subwoofer(50*)
    //^^^this was making the shooter rise. 2.5 good for stage leg

    // double position = vision.area * 10;
        double position = 3;


    // Final target of 200 rot, 0 rps
    TrapezoidProfile.State m_goal = new TrapezoidProfile.State(position, 0); //0.4
    TrapezoidProfile.State m_setpoint = new TrapezoidProfile.State();

    // periodic, update the profile setpoint for 20 ms loop time
    m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);
    // apply the setpoint to the control request
    m_position.Position = m_setpoint.position;
    m_position.Velocity = m_setpoint.velocity;
    angler.setControl(m_position);

    Constants.IntakeSubsystem.ring = false;
    */


    anglerSetpoint();
    // periodic, update the profile setpoint for 20 ms loop time
    m_setpoint = m_profile.calculate(0.020, m_setpoint, m_goal);
    // apply the setpoint to the control request
    m_position.Position = m_setpoint.position;
    m_position.Velocity = m_setpoint.velocity;
    
  }

  public void shootMax(){
    motor1.set(maxVelocity);
    motor2.set(maxVelocity);
    System.out.println("Shoot Max");
  }

  public void shootRPM(){
    VelocityDutyCycle request = new VelocityDutyCycle(3000);
    motor1.setControl(request);
    motor2.setControl(request);

  }

  public void shootStop(){
    motor1.set(0);
    motor2.set(0);
  }


  public double getDistance(){
    return 6*Math.sin(vision.y*Math.PI/180);
  }


  public double getIncrement(){
    return increment;
  }

  public void increaseIncrement(){
    increment+=0.1;
  }
  public void decreaseIncrement(){
    increment-=0.1;
  }


  public double getAvgShooterSpeed(){
    return (motor1.getVelocity().getValueAsDouble() + motor2.getVelocity().getValueAsDouble())/2;
  }

  public void configArray(){
    array[6] = 0.6;
    // array[7] = 
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
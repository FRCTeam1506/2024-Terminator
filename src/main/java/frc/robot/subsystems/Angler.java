// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.MotionMagicDutyCycle;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

public class Angler extends SubsystemBase {
  /** Creates a new Angler. */
  TalonFX motor = new TalonFX(Constants.ShooterSubsystem.AnglerID);
  final MotionMagicVoltage m_motmag = new MotionMagicVoltage(0);
  final MotionMagicDutyCycle m_motmag_2 = new MotionMagicDutyCycle(0);

  public GenericEntry dashangler = Shuffleboard.getTab("Robot").add("setangler", 6)
    .withWidget(BuiltInWidgets.kNumberSlider)
    .withProperties(Map.of("min", 0, "max", 6))
    .getEntry();

  DigitalInput input = Constants.ShooterSubsystem.LimitSwitchDIO;

  boolean hasBeenClickedYet = false;

  public static boolean anglerInUse = false;



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

    // set slot 1 gains to same as slot 0
    var slot1Configs = talonFXConfigs.Slot1;
    slot1Configs.kS = 0.24; // add 0.24 V to overcome friction
    slot1Configs.kV = 0.2; // apply 12 V for a target velocity of 100 rps  //originally 0.12
    // PID runs on position
    slot1Configs.kP = 4.8;
    slot1Configs.kI = 0;
    slot1Configs.kD = 0.1;

    

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
    anglerInUse = true;
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

      if(DriverStation.getAlliance().get() == Alliance.Red){
        dist += Constants.ShooterSubsystem.redOffset; //need to shoot higher on red
      }
      else{
        dist-=0.149; //shoot higher on blue as well //0.089 worked fine for RR
      }
      
      double pos = Constants.ShooterSubsystem.a*Math.pow(dist, 2) - Constants.ShooterSubsystem.b*dist + Constants.ShooterSubsystem.c; //desmos eq, check screenshots 2/21/2024 +++
      motor.setControl(m_motmag.withPosition(pos));

    }
    anglerInUse = false;
  }

  public double getEquationResult(double x){
    return Constants.ShooterSubsystem.a * Math.pow(x, 2) - Constants.ShooterSubsystem.b*x + Constants.ShooterSubsystem.c;
  }

  public void setPositionByPose(){
    double distanceToTarget = TunerConstants.DriveTrain.getState().Pose.minus(Constants.Field.getAllianceSpeakerPose2d()).getTranslation().getNorm();
    double pos = Constants.ShooterSubsystem.a*Math.pow(distanceToTarget, 2) - Constants.ShooterSubsystem.b*distanceToTarget + Constants.ShooterSubsystem.c;
    if(pos > 7){
      pos = 7;
    }
    motor.setControl(m_motmag.withPosition(pos));
  }

  /**
   * Angler function only used in auto to shoot while moving in TTN auto
   */
  public void anglePurposefullyLow(){
    m_motmag.Slot = 0;
    double reduce = 0.45;//0.85

    double xshot = Math.abs(LimelightHelpers.getBotPose_TargetSpace("limelight")[0]);
    double zshot = Math.abs(LimelightHelpers.getBotPose_TargetSpace("limelight")[2]);
    double dist = Math.sqrt(Math.pow(Math.abs(xshot), 2) + Math.pow(Math.abs(zshot), 2)); //pythagorean theorem

    if(Vision.target != 0){
       double pos = Constants.ShooterSubsystem.a*Math.pow(dist, 2) - Constants.ShooterSubsystem.b*dist + Constants.ShooterSubsystem.c; //desmos eq, check screenshots 2/21/2024 +++
       motor.setControl(m_motmag.withPosition(pos - reduce));
    }
  }


  public void setPositionManual(double position){
    anglerInUse = true;
    m_motmag.Slot = 0;
    motor.setControl(m_motmag.withPosition(position));
    anglerInUse = false;
  }

  public void setPositionManualBETA(double position){
    m_motmag.Slot = 1;
    motor.setControl(m_motmag.withPosition(position));
  }

  /**
   * Put the anlger in vertical position when climbing
   */
  public void goVertical(){
    anglerInUse = true;
    m_motmag.Slot = 0;
    double vertPos = 12.5;
    motor.setControl(m_motmag.withPosition(vertPos));
    anglerInUse = false;
  }

  /**
   * Not in use
   */
  public void setPositionIncrement(){
    m_motmag.Slot = 0;
    motor.setControl(m_motmag.withPosition(new ShooterSubsystem().getIncrement()));
  }

  public void anglerUp(){
    anglerInUse = true;
    motor.set(0.3);
  }

  public void anglerDown(){
    anglerInUse = true;
    motor.set(-0.3);
  }

  public void ampPosition(){
    anglerInUse = true;
    m_motmag.Slot = 0;
    motor.setControl(m_motmag.withPosition(3.5)); //4.8   5.05
  }

  public void trapPosition(){
    anglerInUse = true;
    m_motmag.Slot = 0;
    motor.setControl(m_motmag.withPosition(Constants.ShooterSubsystem.anglerSet));//5
    anglerInUse = false;
  }


  public void stopAngler(){
    anglerInUse = false;
    motor.set(0);
    motor.stopMotor();
  }

  public void anglerZero(){
    motor.setPosition(0);
  }

  public void testSwitch(){
    if(!input.get()){
      motor.setPosition(0);
      hasBeenClickedYet = true;
    }
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


  /**
   * Method to make the angler motor coast downward to zero (so that it hits the limit switch) when not in use.
   * This is so that it will zero the position when it hits the limit switch. ---- CURRENTLY UNUSED
   */
  public void coastDownward(){
    if(getPos() > 1 && !Constants.ShooterSubsystem.isShooting){
      m_motmag.Slot = 0;
      motor.setControl(m_motmag.withPosition(0.3).withFeedForward(0.7));//5
    }
  }

  public String getAnglerMotorControlValue(){
    return motor.getControlMode().toString();
  }

  public double getAnglerSlot(){
    return motor.getClosedLoopSlot().getValue();
  }

  /**
   * Note that this does not return the variable of the same name
   * @return boolean of whether the angler is being used by shooting/something else that is not the autotarget function
   */
  public boolean isAnglerBeingUsed(){
    return !motor.getControlMode().toString().equals("NeutralOut ");
  }

  public boolean anglerReadyToShoot(){
    double position = motor.getPosition().getValueAsDouble();

    double xshot = Math.abs(LimelightHelpers.getBotPose_TargetSpace("limelight")[0]);
    double zshot = Math.abs(LimelightHelpers.getBotPose_TargetSpace("limelight")[2]);
    double dist = Math.sqrt(Math.pow(Math.abs(xshot), 2) + Math.pow(Math.abs(zshot), 2)); //pythagorean theorem
    double setpoint = getEquationResult(dist);

    double difference = Math.abs(position - setpoint) / ((position + setpoint) / 2);

    return Math.abs(difference) < 0.05; //plus/minus 3 percent threshold

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.putNumber("angler set", getVisionPosition());
    testSwitch();

    /*

    if((isAnglerBeingUsed() && getAnglerSlot() == 1) || !isAnglerBeingUsed()){

      if(Vision.target > 0){
        double xshot = Math.abs(LimelightHelpers.getBotPose_TargetSpace("limelight")[0]);
        double zshot = Math.abs(LimelightHelpers.getBotPose_TargetSpace("limelight")[2]);
        double dist = Math.sqrt(Math.pow(Math.abs(xshot), 2) + Math.pow(Math.abs(zshot), 2)); //pythagorean theorem

        if(getEquationResult(dist) < 7 && getEquationResult(dist) > 0){
          setPositionManualBETA(getEquationResult(dist));
        }
      }      
      else{
        setPositionManualBETA(0);
      }

    }
    */
    // if(!Constants.ShooterSubsystem.isShooting && getPos() > 2 && hasBeenClickedYet){
    //   coastDownward();
    // }
  }
}

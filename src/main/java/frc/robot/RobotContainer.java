// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.commands.vision.kevin2;
import frc.robot.commands.vision.vision2;
import frc.robot.generated.TunerConstants;

public class RobotContainer {

  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final PS4Controller driver = new PS4Controller(0); // My joystick
  private final PS4Controller operator = new PS4Controller(1); // My joystick

  public static PneumaticHub hub      = new PneumaticHub();
  public static Compressor compressor = new Compressor(1, PneumaticsModuleType.REVPH);

  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public final ArmSubsystem arm = new ArmSubsystem(hub);
  public final Vision vision = new Vision();

  //Commands
  vision2 visionCommand = new vision2(vision);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */
  private Command runAuto = drivetrain.getAutoPath("SWPTestAuto");

  private final Telemetry logger = new Telemetry(MaxSpeed);

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    j.dX.whileTrue(drivetrain.applyRequest(() -> brake));
    j.dTriangle.whileTrue(drivetrain
        .applyRequest(() -> point.withModuleDirection(new Rotation2d(-driver.getLeftY(), -driver.getLeftX()))));

    // reset the field-centric heading on left bumper press
    j.dCircle.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    // if (Utils.isSimulation()) {
    //   drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    // }
    drivetrain.registerTelemetry(logger::telemeterize);

    j.dUp.whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    j.dDown.whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
    j.dLeft.whileTrue(new vision2(vision));
    j.dRight.whileTrue(new kevin2(vision));


    j.oDown.whileTrue(new InstantCommand( () -> arm.setHigh()));
    j.oLeft.whileTrue(new InstantCommand( () -> arm.setLow()));
    j.oRight.whileTrue(new InstantCommand( () -> arm.setMid()));
  }

  public RobotContainer() {
    configureBindings();
    hub.enableCompressorAnalog(100, 120);
    dashboardStuff();
    runCurrentLimits();
  }

  public void dashboardStuff(){
    Pigeon2 gyro = new Pigeon2(50);
    ShuffleboardTab tab = Shuffleboard.getTab("LiftingArm");
    tab.addNumber("PSI", () -> hub.getPressure(0));
    tab.addNumber("Pitch: ", () -> gyro.getPitch().getValueAsDouble() );
    tab.addNumber("Yaw: ", () -> gyro.getYaw().getValueAsDouble());
    tab.addNumber("Roll: ", () -> gyro.getRoll().getValueAsDouble());

  }

  public void runCurrentLimits(){
    int[] swerveMotors = {11,12,21,22,31,32,41,42};
    int[] otherMotors = {60,61};

    TalonFXConfiguration config = new TalonFXConfiguration();

    CurrentLimitsConfigs swerveConfig = new CurrentLimitsConfigs();
    swerveConfig.SupplyCurrentLimitEnable = true;
    swerveConfig.SupplyCurrentLimit = 35;
    swerveConfig.SupplyCurrentThreshold = 60;
    swerveConfig.SupplyTimeThreshold = 0.1;

    
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.StatorCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimit = 50;

    currentConfig.SupplyCurrentLimitEnable = true;
    currentConfig.SupplyCurrentLimit = 35;
    currentConfig.SupplyCurrentThreshold = 60;
    currentConfig.SupplyTimeThreshold = 0.1;

    for(int i: swerveMotors){
      TalonFX motor = new TalonFX(i);
      motor.getConfigurator().apply(swerveConfig);
      motor.close();
    }

    for(int i: otherMotors){
      TalonFX motor = new TalonFX(i);
      motor.getConfigurator().apply(swerveConfig); /// !!!!! CURRENTLY WE ARE NOT DOING STATOR LIMITING ONLY SUPPLY LIMITING !!!!!
      motor.close();
    }

  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return runAuto;
  }
}
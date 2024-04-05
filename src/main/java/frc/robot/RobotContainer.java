// Copyright (c) FIRST and other WPILib contributors. Open Source Software.
// FRC TEAM 1506 --- METAL MUSCLE --- 2024 ROBOT CODE --- TERMINATOR

package frc.robot;

import java.util.HashMap;
import java.util.List;
import java.util.Map;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.ParentDevice;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.Named;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Candle;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Trapper;
import frc.robot.subsystems.Vision;
import frc.robot.commands.angler.setPosition;
import frc.robot.commands.angler.stopAngler;
import frc.robot.commands.autos.toggleGSA;
import frc.robot.commands.autos.autoShooting.stopAnglerIntakeIndexerShooter;
import frc.robot.commands.climber.climberUp;
import frc.robot.commands.climber.individualControl;
import frc.robot.commands.climber.toggleEndGame;
import frc.robot.commands.drivetrain.zeroGyro;
import frc.robot.commands.intake.indexToShoot;
import frc.robot.commands.intake.intake;
import frc.robot.commands.intake.justIndex;
import frc.robot.commands.intake.justIntake;
import frc.robot.commands.intake.stopIndexer;
import frc.robot.commands.intake.stopIntake;
import frc.robot.commands.intake.toggleManualIntake;
import frc.robot.commands.intake.watchIntake;
import frc.robot.commands.shooter.angle;
import frc.robot.commands.shooter.deliverAuto;
import frc.robot.commands.shooter.mailNotes;
import frc.robot.commands.shooter.prepareToShoot;
import frc.robot.commands.shooter.runWheel;
import frc.robot.commands.shooter.shoot;
import frc.robot.commands.shooter.shootAmp;
import frc.robot.commands.shooter.shootAuto;
import frc.robot.commands.shooter.shootConditional;
import frc.robot.commands.shooter.shootIdle;
import frc.robot.commands.shooter.shootOLD;
import frc.robot.commands.shooter.shootPower;
import frc.robot.commands.shooter.shootShufflebaord;
import frc.robot.commands.shooter.shootTrap;
import frc.robot.commands.shooter.PREmailNotes;
import frc.robot.commands.shooter.ShootWhileMoving;
import frc.robot.commands.shooter.stopShooter;
import frc.robot.commands.shooter.toggleAim;
import frc.robot.commands.trapper.sendTrapperHome;
import frc.robot.commands.trapper.trampTheAmp;
import frc.robot.commands.vision.*;
import frc.robot.generated.TunerConstants;

public class RobotContainer {

  private double MaxSpeed = 5; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final PS4Controller driver = new PS4Controller(0); // My joystick
  private final PS4Controller operator = new PS4Controller(1); // My joystick

  //Subsystems!!!!
  public final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain
  public final Vision vision = new Vision(drivetrain);
  public final frc.robot.subsystems.IntakeSubsystem intake = new frc.robot.subsystems.IntakeSubsystem();
  public final frc.robot.subsystems.ShooterSubsystem shooter = new frc.robot.subsystems.ShooterSubsystem();
  public final Angler angler = new Angler();
  public final Autos autos = new Autos(intake, shooter, angler, vision);
  public final Climber climber = new Climber();
  public final Trapper trapper = new Trapper();
  public final Candle candle = new Candle();

  //Commands
  vision2 visionCommand = new vision2(vision);

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * Constants.Swerve.deadband).withRotationalDeadband(MaxAngularRate * Constants.Swerve.deadband) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  /* Path follower */

  // private final Telemetry logger = new Telemetry(MaxSpeed);
  private final Field2d m_field = Constants.Swerve.m_field;

  private void configureBindings() {
    drivetrain.setDefaultCommand( // Drivetrain will execute this command periodically
        drivetrain.applyRequest(() -> drive.withVelocityX(-driver.getLeftY() * MaxSpeed) // Drive forward with
                                                                                           // negative Y (forward)
            .withVelocityY(-driver.getLeftX() * MaxSpeed) // Drive left with negative X (left)
            .withRotationalRate(-driver.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
        ).ignoringDisable(true));

    //idle shooter
    // shooter.setDefaultCommand(new shootIdle(shooter));
    // climber.setDefaultCommand(new individualControl(climber));

    j.dA.whileTrue(drivetrain.applyRequest(() -> brake));

    // zero gyro
    j.dB.onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(90)));
    }
    // drivetrain.registerTelemetry(logger::telemeterize);

    // j.dUp.whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(0.5).withVelocityY(0)));
    // j.dDown.whileTrue(drivetrain.applyRequest(() -> forwardStraight.withVelocityX(-0.5).withVelocityY(0)));
    
    // j.dLeft.whileTrue(new align(vision)); //shooter alignment
    // j.dLeft.whileTrue(new vision2(vision).until(() -> vision.x > -Constants.Limelight.shooterThreshold && vision.x < Constants.Limelight.shooterThreshold)); //shooter alignment

    j.dLeft.whileTrue(new align(vision)); //shooter alignment
    // j.dLeft.whileTrue(new align(vision));
    // j.dRight.whileTrue(new kevin2(vision)); //intake alignment

    //ACTUAL CONTROLS!!!

    //intake
    j.oRT.whileTrue(new RepeatCommand(new InstantCommand(() -> intake.intake())));
    j.oRT.whileFalse(new InstantCommand(() -> intake.stopIntakeWithReset()));
    j.oLT.whileTrue(new InstantCommand(() -> intake.outtake()));
    j.oLT.whileFalse(new InstantCommand(() -> intake.stopIntake()));

    //just in case
    j.oShare.whileTrue(new toggleManualIntake());

    //delete
    j.dRT.whileTrue(new RepeatCommand(new InstantCommand(() -> intake.intake())));
    j.dRT.whileFalse(new InstantCommand(() -> intake.stopIntakeWithReset()));
    j.dLT.whileTrue(new InstantCommand(() -> intake.outtake()));
    j.dLT.whileFalse(new InstantCommand(() -> intake.stopIntake()));

    j.dY.whileTrue(new toggleGSA());


    //manual shooter
    // j.oRB.whileTrue(new InstantCommand(() -> shooter.shootMax()));
    // j.oRB.whileFalse(new InstantCommand(() -> shooter.shootStop()));

    //manual angler
    j.dUp.whileTrue(new InstantCommand(() -> angler.anglerUp()));
    j.dDown.whileTrue(new InstantCommand(() -> angler.anglerDown()));
    // j.dPS.whileTrue(new InstantCommand(() -> angler.anglerZero())); //limit switch replcaed this functionality
    j.dUp.whileFalse(new InstantCommand(() -> angler.stopAngler()));
    j.dDown.whileFalse(new InstantCommand(() -> angler.stopAngler()));
    
    //auto shooting
    j.oRight.whileTrue(new shootConditional(shooter, intake, angler, vision, trapper));
    j.oRight.whileFalse(new InstantCommand(() -> shooter.shootStop()));
    j.oRight.whileFalse(new InstantCommand(() -> intake.stopIndexer()));
    j.oRight.whileFalse(new InstantCommand(() -> angler.stopAngler()));
    j.oRight.whileFalse(new InstantCommand(() -> trapper.stopTrapper()));

    // j.dRight.whileTrue(new shootShufflebaord(shooter, intake, angler, vision)); //for testing regression

    // j.dRight.whileTrue(new shoot(shooter, intake, angler, vision));
    // j.dRight.whileFalse(new InstantCommand(() -> shooter.shootStop()));
    // j.dRight.whileFalse(new InstantCommand(() -> intake.stopIndexer()));
    // j.dRight.whileFalse(new InstantCommand(() -> angler.stopAngler()));

    j.dRight.whileTrue(new shootConditional(shooter, intake, angler, vision, trapper));
    j.dRight.whileFalse(new InstantCommand(() -> shooter.shootStop()));
    j.dRight.whileFalse(new InstantCommand(() -> intake.stopIndexer()));
    j.dRight.whileFalse(new InstantCommand(() -> angler.stopAngler()));
    j.dRight.whileFalse(new InstantCommand(() -> trapper.stopTrapper()));


    j.oTouchpad.whileTrue(new shootAmp(angler, shooter, intake));
    j.oTouchpad.whileFalse(new InstantCommand(() -> shooter.shootStop()));
    j.oTouchpad.whileFalse(new InstantCommand(() -> intake.stopIndexer()));
    j.oTouchpad.whileFalse(new InstantCommand(() -> angler.stopAngler()));

    j.dTouchpad.whileTrue(new toggleAim());

    //switch to endgame mode
    j.dR3.whileTrue(new toggleEndGame());

    //manual shooting
    j.oX.whileTrue(new shootPower(shooter, intake, angler, vision, 3)); //dashangler.getDouble(6))original angler setpoint 3.15 //2.6 og //2.9 for blue
    j.oX.whileFalse(new InstantCommand(() -> shooter.shootStop()));
    j.oX.whileFalse(new InstantCommand(() -> intake.stopIndexer()));
    j.oX.whileFalse(new InstantCommand(() -> angler.stopAngler()));

    j.oR3.whileTrue(new PREmailNotes(shooter, intake, angler, vision, Constants.ShooterSubsystem.mailNotesPosition));
    j.oR3.whileFalse((new indexToShoot(intake).withTimeout(0.2)).andThen(new stopAnglerIntakeIndexerShooter(angler, intake, shooter)));

    // j.oR3.whileTrue(new mailNotes(shooter, intake, angler, vision, Constants.ShooterSubsystem.mailNotesPosition));
    // j.oR3.whileFalse(new InstantCommand(() -> shooter.shootStop()));
    // j.oR3.whileFalse(new InstantCommand(() -> intake.stopIndexer()));
    // j.oR3.whileFalse(new InstantCommand(() -> angler.stopAngler()));



    //climber
    // j.oY.whileTrue(new RepeatCommand(new InstantCommand(() -> climber.up())));
    j.oY.whileTrue(new RepeatCommand(new climberUp(climber)));
    j.oY.whileTrue(new InstantCommand(() -> angler.goVertical()));
    j.oA.whileTrue(new RepeatCommand(new InstantCommand(() -> climber.down())));
    j.oY.whileFalse(new InstantCommand(() -> climber.stop()));
    j.oA.whileFalse(new InstantCommand(() -> climber.stop()));
    j.oL3.whileTrue(new InstantCommand(() -> climber.zeroClimber()));
    
    

    //increment testing
    // j.dShare.whileTrue(new InstantCommand(() -> shooter.decreaseIncrement()));
    // j.dOptions.whileTrue(new InstantCommand(() -> shooter.increaseIncrement()));
    // j.oTouchpad.whileTrue(new angle(angler));
    // j.oTouchpad.whileFalse(new InstantCommand(() -> angler.stopAngler()));

    //trapper
    j.oUp.whileTrue(new InstantCommand(() -> trapper.up()));
    j.oDown.whileTrue(new InstantCommand(() -> trapper.down()));
    // j.oRB.whileTrue(new InstantCommand(() -> trapper.trapPosition())); //no jogging for new trapper design
    j.oRB.whileTrue(new InstantCommand(() -> trapper.intake()));
    j.oLB.whileTrue(new InstantCommand(() -> trapper.intake()));
    j.oLB.whileTrue(new InstantCommand(() -> intake.outtake()));
    j.oLB.whileFalse(new InstantCommand(() -> intake.stopIntake()));
    j.oB.whileTrue(new InstantCommand(() -> trapper.shootTrap()));
    j.dRB.whileTrue(new InstantCommand(() -> trapper.intake()));
  

    j.oUp.whileFalse(new InstantCommand(() -> trapper.stopTrapper())); //send trapper home
    j.oDown.whileFalse(new InstantCommand(() -> trapper.stopTrapper()));
    j.oRB.whileFalse(new InstantCommand(() -> trapper.stopTrapper()));
    // j.oRB.whileFalse(new sendTrapperHome(trapper));
    j.dRB.whileFalse(new InstantCommand(() -> trapper.stopTrapper()));
    j.oLB.whileFalse(new InstantCommand(() -> trapper.stopTrapper()));
    j.oB.whileFalse(new InstantCommand(() -> trapper.stopTrapper()));


    //trapper amp command
    // j.oPS.whileTrue(new InstantCommand(() -> trapper.setAmpPosition()));
    j.oPS.whileTrue(new trampTheAmp(trapper, shooter, intake));
    j.oPS.whileFalse(new InstantCommand(() -> trapper.sendTrapperHome()));
    j.oPS.whileFalse(new InstantCommand(() -> trapper.stopIntake()));

    //potential trap shooting code line 1 and 3
    // j.dRB.whileTrue(new shootTrap(angler, shooter, intake, vision));
    // j.dX.whileTrue(new RepeatCommand(new forwardTest(vision)));
    // j.dX.whileTrue(new forwardTest(vision));

    j.dX.whileFalse(new InstantCommand(() -> shooter.shootStop()));
    j.dX.whileFalse(new InstantCommand(() -> intake.stopIndexer()));
    j.dX.whileFalse(new InstantCommand(() -> angler.stopAngler()));

  
    //trapper in endgame
    j.oLeft.whileTrue(new InstantCommand(() -> trapper.shootTrap()));
    j.oLeft.whileFalse(new InstantCommand(() -> trapper.stopTrapper()));

    j.dPS.whileTrue(new InstantCommand(() -> trapper.verticalZero()));
  }

  private final intake intakecommand = new intake(intake);

  public RobotContainer() {
    // NamedCommands.registerCommands(map);
    NamedCommands.registerCommand("Intake", new intake(intake).until(() -> Constants.IntakeSubsystem.irNine.get() == false));
    NamedCommands.registerCommand("StartIntake", new justIntake(intake));
    NamedCommands.registerCommand("StopIntake", new stopIntake(intake));
    NamedCommands.registerCommand("Watch Intake", new watchIntake(intake));
    NamedCommands.registerCommand("ZeroGyro", new zeroGyro().withTimeout(0.1));

    NamedCommands.registerCommand("Shoot", new shootOLD(shooter, intake, angler, vision));
    NamedCommands.registerCommand("ShootWhileMoving", new ShootWhileMoving(shooter, intake, angler, vision, 1.3));//1.2
    NamedCommands.registerCommand("PrepareToShoot", new prepareToShoot(angler, shooter, intake, getAutoSetpoint(4.17)));//1.2
    NamedCommands.registerCommand("PrepareToShootUnderStage", new prepareToShoot(angler, shooter, intake, getAutoSetpoint(4.36)));//1.2
    NamedCommands.registerCommand("PrepareToShootMidStage", new prepareToShoot(angler, shooter, intake, getAutoSetpoint(4)));//for use in GoFar auto
    NamedCommands.registerCommand("IndexToShoot", new indexToShoot(intake).alongWith(new runWheel(shooter).withTimeout(0.05)).withTimeout(0.3).andThen(new stopShooter(shooter).withTimeout(0.05), new stopIntake(intake), new stopIndexer(intake)).withTimeout(0.1));//1.2
    NamedCommands.registerCommand("JustIndexAndShoot", new justIndex(intake).alongWith(new runWheel(shooter), new justIntake(intake)));
    NamedCommands.registerCommand("RunShooter", new runWheel(shooter).withTimeout(0.05));

    NamedCommands.registerCommand("ShootLine", new shootAuto(shooter, intake, angler, vision, 0.7));
    NamedCommands.registerCommand("ShootStage", new shootAuto(shooter, intake, angler, vision, getAutoSetpoint(2.94)));
    NamedCommands.registerCommand("ShootBlackLine", new shootAuto(shooter, intake, angler, vision, getAutoSetpoint(2.69)));
    NamedCommands.registerCommand("ShootMidStage", new shootAuto(shooter, intake, angler, vision, getAutoSetpoint(4))); // og 0.725 //then 0.785 //then 0.885
    NamedCommands.registerCommand("ShootAmp", new shootAuto(shooter, intake, angler, vision, getAutoSetpoint(1.89)));
    NamedCommands.registerCommand("DeliverAuto", new deliverAuto(shooter, intake, angler, 0.5));
    NamedCommands.registerCommand("DeliverAutoSoftly", new deliverAuto(shooter, intake, angler,0.15));
    NamedCommands.registerCommand("StopEverything", new ParallelCommandGroup(new stopShooter(shooter), new stopAngler(angler), new stopIntake(intake), new stopIndexer(intake)).withTimeout(0.1));
    NamedCommands.registerCommand("ShootBase", new shootAuto(shooter, intake, angler, vision, getAutoSetpoint(0.88))); //5.6 //5.75 //0.95 b4 states

    NamedCommands.registerCommand("AutoNotePost_PTS", new setPosition(angler, getAutoSetpoint(3.03)));
    NamedCommands.registerCommand("AutoNoteCenterNAmp_PTS", new setPosition(angler, getAutoSetpoint(3)));
    NamedCommands.registerCommand("AmpAuto_PTS", new setPosition(angler, getAutoSetpoint(3.8)).alongWith(new intake(intake), new runWheel(shooter)).andThen(new stopAnglerIntakeIndexerShooter(angler, intake, shooter)));
    // NamedCommands.registerCommand("AutoNoteCenter_PTS", new prepareToShoot(angler, shooter, intake, 2.43));



    configureBindings();
    dashboardStuff();
    runCurrentLimits();

    // drivetrain.configurePathPlanner();

    // autos.registerCommands();
    autos.getAutos();

    //our lovely field
    // SmartDashboard.putData("Field", Constants.Swerve.m_field);

    // ParentDevice.optimizeBusUtilizationForAll(new ParentDevice[])

  }

  public void dashboardStuff(){
    ShuffleboardTab tab = Shuffleboard.getTab("Robot");
    tab.addNumber("Pitch: ", () -> TunerConstants.DriveTrain.getPigeon2().getPitch().getValueAsDouble() );
    tab.addNumber("Yaw: ", () -> TunerConstants.DriveTrain.getPigeon2().getRotation2d().getDegrees());
    tab.addNumber("Roll: ", () -> TunerConstants.DriveTrain.getPigeon2().getRoll().getValueAsDouble());
    // tab.addNumber("Necessary Angle", () -> Math.toDegrees(Math.atan(66/(Vision.z * 39.37)))/5.14);
    tab.addNumber("Intake Torque Current", () -> intake.getTorqueCurrent());
    tab.addBoolean("AutoAim", () -> Constants.ShooterSubsystem.autoAim);
    tab.addBoolean("Auto Intake", () -> !Constants.IntakeSubsystem.manualIntake);
    tab.addBoolean("End Game", () -> Constants.ClimberSubsystem.endGame);
    //shuffleboard for the angler position with a slider
    
  }

  public void runCurrentLimits(){
    int[] swerveMotors = {11,12,21,22,31,32,41,42};
    int[] otherMotors = {51,57,52,60,61,62,55,56}; //does not include 58

    CurrentLimitsConfigs swerveConfig = new CurrentLimitsConfigs();
    swerveConfig.SupplyCurrentLimitEnable = true;
    swerveConfig.SupplyCurrentLimit = 35;
    swerveConfig.SupplyCurrentThreshold = 60;
    swerveConfig.SupplyTimeThreshold = 0.1;

    //Stator Limiting Not Being Used
    CurrentLimitsConfigs currentConfig = new CurrentLimitsConfigs();
    currentConfig.StatorCurrentLimitEnable = true;
    currentConfig.StatorCurrentLimit = 100;

    // currentConfig.SupplyCurrentLimitEnable = true;
    // currentConfig.SupplyCurrentLimit = 38;
    // currentConfig.SupplyCurrentThreshold = 60;
    // currentConfig.SupplyTimeThreshold = 0.1;
    

    // for(int i: swerveMotors){
    //   TalonFX motor = new TalonFX(i);
    //   motor.getConfigurator().apply(currentConfig);
    //   motor.optimizeBusUtilization();
    //   motor.close();
    // }

    for(int i: otherMotors){
      TalonFX motor = new TalonFX(i);
      motor.getConfigurator().apply(currentConfig); /// !!!!! CURRENTLY WE ARE NOT DOING STATOR LIMITING ONLY SUPPLY LIMITING !!!!!
      // motor.optimizeBusUtilization();
      motor.close();
    }

    TalonFX motor = new TalonFX(58);
    motor.getConfigurator().apply(currentConfig.withStatorCurrentLimit(120));
    motor.close();

  }


  public Command getAutonomousCommand() {


    // autos.registerCommands();

    /*
    List<Translation2d> bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(0.0, 0.0, Rotation2d.fromDegrees(0)),
        new Pose2d(20.0, 20.0, Rotation2d.fromDegrees(0))

    );

    PathPlannerPath path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(5.0, 5.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(3.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );
    */

    /* First put the drivetrain into auto run mode, then run the auto */
    // return drivetrain.followPathCommand(path);

    // Command goForward = drivetrain.getAutoPath("Forward");
    // return goForward;
    return autos.sendAutos();
    // return new PathPlannerAuto("GoFar");
    // return new PathPlannerAuto("ThreadTheNeedle");
  }

  public double getAutoSetpoint(double distance){
    if(DriverStation.getAlliance().isPresent()){
      if(DriverStation.getAlliance().get() == Alliance.Red){
        double d = distance + Constants.ShooterSubsystem.redOffset;
        return angler.getEquationResult(d);
      }
    }
    return angler.getEquationResult(distance);
  }

  public Command stopEverything(){
    return new ParallelCommandGroup(new stopShooter(shooter), new stopAngler(angler), new stopIntake(intake), new stopIndexer(intake)).withTimeout(0.1);
  } 

  public void periodic(){
    // Do this in either robot or subsystem init
     SmartDashboard.putData("Field", m_field);
    j.operator.setRumble(RumbleType.kRightRumble, 1);
    // Do this in either robot periodic or subsystem periodic ---- odometry
    m_field.setRobotPose(drivetrain.getState().Pose); ////say TunerConstants.DriveTrain.getState().Pose or something like that
    //m_field.setRobotPose(Vision.estimator.getEstimatedPosition().getX(), Vision.estimator.getEstimatedPosition().getY(), Vision.estimator.getEstimatedPosition().getRotation());
    // if(drivetrain.getPigeon2().getStickyFaultField().getValue() > 0){
    //   drivetrain.getPigeon2().clearStickyFaults();
    //   // drivetrain.getPigeon2().clear
    // }
  }
}
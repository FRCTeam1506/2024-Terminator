// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;

public class alignPID extends Command {
  /** Creates a new alignPID. */
  Vision vision;
  static double initialX;
  static double initialYaw;
  static double gyroGoal;

  HolonomicDriveController controller = new HolonomicDriveController(
    new PIDController(1, 0, 0), new PIDController(1, 0, 0),
    new ProfiledPIDController(10, 0, 0,
      new TrapezoidProfile.Constraints(5.28, 3.14)));

  ChassisSpeeds stop = new ChassisSpeeds(0,0, 0);
  SwerveRequest.ApplyChassisSpeeds request;


  public alignPID(Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    request = new SwerveRequest.ApplyChassisSpeeds();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialX = LimelightHelpers.getTX("limelight");
    initialYaw = TunerConstants.DriveTrain.getState().Pose.getRotation().getDegrees();
    gyroGoal = initialYaw - initialX;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    Pose2d goalPose = new Pose2d(TunerConstants.DriveTrain.getState().Pose.getX(), TunerConstants.DriveTrain.getState().Pose.getY(), Rotation2d.fromDegrees(gyroGoal));

    ChassisSpeeds chosenSpeeds = controller.calculate(
      TunerConstants.DriveTrain.getState().Pose, goalPose, 0, Rotation2d.fromDegrees(gyroGoal));
        
    ChassisSpeeds speedTest = new ChassisSpeeds(0, 0, 3);

    
      // TunerConstants.DriveTrain.applyRequest(() -> request.withSpeeds(speedTest));
      TunerConstants.DriveTrain.setControl(request.withSpeeds(chosenSpeeds));
      System.out.println(chosenSpeeds.omegaRadiansPerSecond);
    // if(!isFinished()) {
    //   double speed = controller.calculate(vision.zshot, 0, Timer.getFPGATimestamp());
    //   TunerConstants.DriveTrain.applyRequest(() -> request.withSpeeds(new ChassisSpeeds(0, 0, -speed)));
    // }else{
    //   TunerConstants.DriveTrain.applyRequest(() -> request.withSpeeds(stop));
    // }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    TunerConstants.DriveTrain.applyRequest(() -> request.withSpeeds(stop));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Vision.x) < 2;
    // return false;
    // return controller.atReference();
  }
}
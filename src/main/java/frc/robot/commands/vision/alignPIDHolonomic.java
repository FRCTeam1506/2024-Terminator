// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;

public class alignPIDHolonomic extends Command {
  /** Creates a new alignPID. */
  Vision vision;
  HolonomicDriveController controller;

  double visionOffset, initialYaw, gyroGoal;
  Rotation2d rotationGoal;

  PIDController pidcontroller = new PIDController(1, 0, 0);
  ProfiledPIDController profiledController = new ProfiledPIDController(1, 0, 0, new Constraints(2, 1));

  ChassisSpeeds stop = new ChassisSpeeds(0,0, 0);
  SwerveRequest.ApplyChassisSpeeds request = new SwerveRequest.ApplyChassisSpeeds();


  public alignPIDHolonomic(Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    controller = new HolonomicDriveController(pidcontroller, pidcontroller, profiledController);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialYaw = TunerConstants.DriveTrain.getPigeon2().getAngle();
    visionOffset = vision.zshot;
    gyroGoal = initialYaw - visionOffset;
    rotationGoal = new Rotation2d(gyroGoal);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(!isFinished()) {
      // double speed = controller.calculate(vision.zshot, 1,  Timer.getFPGATimestamp());
      Pose2d currentPose = TunerConstants.DriveTrain.getState().Pose;
      // Trajectory.State state = new 
      Pose2d desiredPose = new Pose2d(TunerConstants.DriveTrain.getState().Pose.getX(), TunerConstants.DriveTrain.getState().Pose.getY(), rotationGoal);
      ChassisSpeeds speeds = controller.calculate(currentPose, desiredPose, 1, rotationGoal);
      TunerConstants.DriveTrain.applyRequest(() -> request.withSpeeds(speeds.times(-1)));
    }else{
      TunerConstants.DriveTrain.applyRequest(() -> request.withSpeeds(stop));
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    TunerConstants.DriveTrain.applyRequest(() -> request.withSpeeds(stop));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(Vision.zshot) < 0.3;
  }
}

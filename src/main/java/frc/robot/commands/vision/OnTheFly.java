// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import frc.robot.commands.drivetrain.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;

import java.util.List;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.Swerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;

public class OnTheFly extends Command {

  Vision vision;
  CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  double x,y,theta;
  double angularSpeed = Math.PI / 2;
  boolean finished = false;
  Pigeon2 gyro = TunerConstants.DriveTrain.getPigeon2();

  double initialX, initialYaw, gyroGoal;

  List<Translation2d> bezierPoints;
  PathPlannerPath path;


  public OnTheFly(Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // if(Vision.target == 0){
    //   finished = true;
    // }
    // else{
    //   finished = false;
    // }
    
    initialX = Vision.x;
    initialYaw = gyro.getYaw().getValueAsDouble();
    gyroGoal = initialYaw - initialX;


    bezierPoints = PathPlannerPath.bezierFromPoses(
        new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0)),
        new Pose2d(2.0, 2.0, Rotation2d.fromDegrees(0))

    );

    path = new PathPlannerPath(
            bezierPoints,
            new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI), // The constraints for this path. If using a differential drivetrain, the angular constraints have no effect.
            new GoalEndState(0.0, Rotation2d.fromDegrees(-90)) // Goal end state. You can set a holonomic rotation here. If using a differential drivetrain, the rotation will have no effect.
    );

    // Prevent the path from being flipped if the coordinates are already correct
    path.preventFlipping =true;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // AutoBuilder.followPath(path);
    TunerConstants.DriveTrain.followPathCommand(path);
    
    System.out.println("KEVIN!!!!");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import frc.robot.commands.drivetrain.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants.Swerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;

public class vision2 extends Command {
  /** Creates a new vision2. */
  Vision vision;
  CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain;
  SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  double x,y,theta;
  double angularSpeed = Math.PI / 2;
  boolean finished = false;
  double threshold = 5;
  Pigeon2 gyro = new Pigeon2(50);

  double initialX, initialYaw, gyroGoal;



  public vision2(Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialX = Vision.x;
    initialYaw = gyro.getYaw().getValueAsDouble();
    gyroGoal = initialYaw - initialX;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speedsLeft = new ChassisSpeeds(0,0, angularSpeed);
    ChassisSpeeds speedsRight = new ChassisSpeeds(0,0, -angularSpeed);
    SwerveRequest.ApplyChassisSpeeds request = new SwerveRequest.ApplyChassisSpeeds();

    if(gyro.getYaw().getValueAsDouble() < gyroGoal + threshold && gyro.getYaw().getValueAsDouble() > gyroGoal-threshold){
      finished = true;
    }
    else{
      finished = false; // otherwise the true carries over from the last alignment
    }

    if(gyro.getYaw().getValueAsDouble() > gyroGoal + threshold){
      System.out.println("Too far left");
      drivetrain.setControl(request.withSpeeds(speedsRight));

    }

    else if(gyro.getYaw().getValueAsDouble() < gyroGoal - threshold){
      System.out.println("Too far right");
      drivetrain.setControl(request.withSpeeds(speedsLeft));
    }

    
    // drivetrain.setControl(request.withSpeeds(speeds));
    
/* 
    if(Vision.target == 0){
      finished = true;
    }

    else if(Vision.x < threshold && Vision.x > -threshold){
      finished = true;
    }

    else if(Vision.x > threshold){
      drivetrain.applyRequest(() -> forwardStraight.withRotationalRate(Math.PI/4)).until(() -> Vision.x < threshold && Vision.x > -threshold);
      System.out.println("Too far left");
      drivetrain.setControl(request.withSpeeds(speedsRight));

    }

    else if(Vision.x < threshold){
      System.out.println("Too far right");
      drivetrain.setControl(request.withSpeeds(speedsLeft));
    }

*/


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

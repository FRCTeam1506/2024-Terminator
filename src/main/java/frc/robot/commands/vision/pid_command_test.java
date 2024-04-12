// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.generated.TunerConstants;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class pid_command_test extends ProfiledPIDCommand {
  /** Creates a new pid_command_test. */

  static double initialX = LimelightHelpers.getTX("limelight");
  static double initialYaw = TunerConstants.DriveTrain.getState().Pose.getRotation().getDegrees();
  static double gyroGoal = initialYaw - initialX;
  public static double speedOutput;

  public pid_command_test() {
    super(
        // The ProfiledPIDController used by the command
        new ProfiledPIDController(
            // The PID gains
            Constants.Swerve.driveP,
            0,
            0,
            // The motion profile constraints
            new TrapezoidProfile.Constraints(3, 2)),
        // This should return the measurement
        () -> TunerConstants.DriveTrain.getState().Pose.getRotation().getDegrees(),
        // This should return the goal (can also be a constant)
        () -> new TrapezoidProfile.State(gyroGoal, 0),
        // This uses the output
        (output, setpoint) -> {
          // Use the output (and setpoint, if desired) here
          TunerConstants.DriveTrain.setControl(new SwerveRequest.ApplyChassisSpeeds().withSpeeds(new ChassisSpeeds(0, 0, output.doubleValue())));
          speedOutput = output.doubleValue();
          System.out.println(speedOutput);
        });
    // Use addRequirements() here to declare subsystem dependencies.
    // Configure additional PID options by calling `getController` here.
    // getController().setTolerance(3);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return getController().atSetpoint();
  }
}

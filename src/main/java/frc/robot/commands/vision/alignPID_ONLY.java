package frc.robot.commands.vision;

import frc.robot.commands.drivetrain.*;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.SwerveControlRequestParameters;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Swerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;

public class alignPID_ONLY extends Command {
  /** Creates a new vision2. */
  Vision vision;
  // SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle();
  double x,y,theta;
  boolean finished = false;
  double threshold = 3;  

  double initialX, initialArea, initialYaw, gyroGoal;

  SwerveRequest.ApplyChassisSpeeds request2 = new SwerveRequest.ApplyChassisSpeeds();


  public alignPID_ONLY(Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // initialX = Vision.x;
    finished = false;

    initialX = LimelightHelpers.getTX("limelight");
    // initialYaw = TunerConstants.DriveTrain.getPigeon2().getAngle();
    initialYaw = TunerConstants.DriveTrain.getState().Pose.getRotation().getDegrees();
    gyroGoal = initialYaw - initialX;

    if((Math.abs(TunerConstants.DriveTrain.getState().Pose.getRotation().getDegrees() - gyroGoal) < 2)){
      finished = true;
    }

    System.out.println(initialX);
    System.out.println(initialYaw);
    System.out.println(gyroGoal);
    System.out.println(gyroGoal % 360);

    
    // Pose2d targetPose2d = new Pose2d(0,0,Rotation2d.fromDegrees(gyroGoal));
    // System.out.println(targetPose2d.getRotation());
    // System.out.println(TunerConstants.DriveTrain.getPigeon2().getAngle() + " gyro angle");
    // System.out.println(Vision.zshot);

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SwerveRequest.FieldCentricFacingAngle request = new SwerveRequest.FieldCentricFacingAngle();

    // request.HeadingController.setPID(0.8, 0.0025, 0.0);
    request.HeadingController.setPID(Constants.Swerve.driveP-4, 0.0, Constants.Swerve.driveD);
    request.Deadband = 3;
    request.RotationalDeadband = 2;
    TunerConstants.DriveTrain.setControl(request.withTargetDirection(Rotation2d.fromDegrees(gyroGoal % 360)));
    // finished = true;
    // TunerConstants.DriveTrain.setControl(request);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds stop = new ChassisSpeeds(0,0, 0);
    SwerveRequest.ApplyChassisSpeeds request = new SwerveRequest.ApplyChassisSpeeds();
    TunerConstants.DriveTrain.setControl(request.withSpeeds(stop));

    // TunerConstants.DriveTrain.setControl(request.withTargetDirection(Rotation2d.fromDegrees(185)));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // if(LimelightHelpers.getTV("limelight")){
    //   if(Math.abs(LimelightHelpers.getTX("limelight")) < 3){
    //     return true;
    //   }
    // }
    // return finished;//Math.abs(Vision.x) < 3;

    // if(Math.abs(TunerConstants.DriveTrain.getState().Pose.getRotation().getDegrees() - gyroGoal) < 2){
    //   return true;
    // }
    // return false;
    return (Math.abs(TunerConstants.DriveTrain.getState().speeds.omegaRadiansPerSecond) < 0.2 && (Math.abs(TunerConstants.DriveTrain.getState().Pose.getRotation().getDegrees() - gyroGoal) < 2)) || finished;
  }
}

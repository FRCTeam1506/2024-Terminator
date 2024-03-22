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

public class forwardTest extends Command {
  /** Creates a new vision2. */
  Vision vision;
  SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  double threshold = 0.03;
  double speed = 0.25;
  boolean finished = false;
  boolean tooFar;


  public forwardTest(Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    vision.stagePipeline();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    vision.stagePipeline();
    finished = false;
    if(vision.shotdist > 0.88 + threshold){
      tooFar = true;
    }
    else if(vision.shotdist < 0.88 - threshold){
      tooFar = false;
    }
    else{
      finished = true;
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    ChassisSpeeds backward = new ChassisSpeeds(speed, 0,0);
    ChassisSpeeds forward = new ChassisSpeeds(-speed, 0,0);

    ChassisSpeeds stop = new ChassisSpeeds(0,0, 0);
    SwerveRequest.ApplyChassisSpeeds request = new SwerveRequest.ApplyChassisSpeeds();

    // if(Vision.shotdist > 0.88 + threshold){
    //   TunerConstants.DriveTrain.setControl(request.withSpeeds(forward));
    // }
    // else if(Vision.shotdist < 0.88 - threshold){
    //   TunerConstants.DriveTrain.setControl(request.withSpeeds(backward));
    // }
    // else{
    //   TunerConstants.DriveTrain.setControl(request.withSpeeds(stop));
    //   finished = true;
    // }

    if(tooFar && vision.shotdist > 0.89){
      TunerConstants.DriveTrain.setControl(request.withSpeeds(forward));
    }
    else if (!tooFar && vision.shotdist < 0.87) {
      TunerConstants.DriveTrain.setControl(request.withSpeeds(backward));
    }
    else{
      finished = true;
      TunerConstants.DriveTrain.setControl(request.withSpeeds(stop));
    }

    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds stop = new ChassisSpeeds(0,0, 0);
    SwerveRequest.ApplyChassisSpeeds request = new SwerveRequest.ApplyChassisSpeeds();
    TunerConstants.DriveTrain.setControl(request.withSpeeds(stop));
    vision.defaultPipeline();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
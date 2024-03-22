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

public class stageAlign extends Command {
  /** Creates a new vision2. */
  Vision vision;
  SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  double x,y,theta;
  double angularSpeed = Math.PI / 3.5;  // pi/4 at kettering week 1
  double farAngularSpeed = Math.PI / 5;  //pi/6 at kettering week 1
  boolean finished = false;
  double threshold = 4;

  double initialX, initialArea, initialYaw, gyroGoal;

  double targetDistance = 0.87;
  double fowardThreshold = 0.06;
  boolean forwardFinished = false;



  public stageAlign(Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialX = Vision.x;
    initialArea = Vision.area;
    initialYaw = TunerConstants.DriveTrain.getPigeon2().getYaw().getValueAsDouble();
    gyroGoal = initialYaw - initialX;
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds slowLeft = new ChassisSpeeds(0,0, farAngularSpeed/3*2);
    ChassisSpeeds slowRight = new ChassisSpeeds(0,0, -farAngularSpeed/3*2);

    ChassisSpeeds forward = new ChassisSpeeds(2, 0,0);
    ChassisSpeeds backward = new ChassisSpeeds(-2, 0,0);

    ChassisSpeeds stop = new ChassisSpeeds(0,0, 0);
    SwerveRequest.ApplyChassisSpeeds request = new SwerveRequest.ApplyChassisSpeeds();

    
    if(TunerConstants.DriveTrain.getPigeon2().getYaw().getValueAsDouble() < gyroGoal + threshold && TunerConstants.DriveTrain.getPigeon2().getYaw().getValueAsDouble() > gyroGoal-threshold){
      TunerConstants.DriveTrain.setControl(request.withSpeeds(stop));
      finished = true;
    }
    else{
      finished = false; // otherwise the true carries over from the last alignment
    }
    if(!(Vision.shotdist > targetDistance - fowardThreshold && Vision.shotdist < targetDistance + fowardThreshold)){
      forwardFinished = false;
    }
    else{
      forwardFinished = true;
    }

    //Aligning the robot with the speaker apriltag
    if(TunerConstants.DriveTrain.getPigeon2().getYaw().getValueAsDouble() > gyroGoal + threshold){
      TunerConstants.DriveTrain.setControl(request.withSpeeds(slowRight));
    }

    //Aligning the robot with the speaker apriltag
    else if(TunerConstants.DriveTrain.getPigeon2().getYaw().getValueAsDouble() < gyroGoal - threshold){
      TunerConstants.DriveTrain.setControl(request.withSpeeds(slowLeft));
    }


    //forward alignment
    if(Vision.shotdist > targetDistance + fowardThreshold && !forwardFinished){
      TunerConstants.DriveTrain.setControl(request.withSpeeds(forward));
    }

    else if(Vision.shotdist < targetDistance - fowardThreshold && !forwardFinished){
      TunerConstants.DriveTrain.setControl(request.withSpeeds(backward));
    }

    else if(Vision.shotdist > targetDistance - fowardThreshold && Vision.shotdist < targetDistance + fowardThreshold){
      // TunerConstants.DriveTrain.setControl(request.withSpeeds(stop));
      forwardFinished = true;
    }


  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    ChassisSpeeds stop = new ChassisSpeeds(0,0, 0);
    SwerveRequest.ApplyChassisSpeeds request = new SwerveRequest.ApplyChassisSpeeds();
    TunerConstants.DriveTrain.setControl(request.withSpeeds(stop));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished = forwardFinished = true;
  }
}
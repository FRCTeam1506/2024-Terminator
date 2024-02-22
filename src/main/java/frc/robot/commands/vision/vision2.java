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
  SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  double x,y,theta;
  double angularSpeed = Math.PI / 4;
  double farAngularSpeed = Math.PI / 6;
  boolean finished = false;
  double threshold = 4;
  Pigeon2 gyro = new Pigeon2(50);

  double initialX, initialArea, initialYaw, gyroGoal;



  public vision2(Vision vision) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    initialX = Vision.x;
    initialArea = Vision.area;
    initialYaw = gyro.getYaw().getValueAsDouble();
    gyroGoal = initialYaw - initialX;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    ChassisSpeeds speedsLeft = new ChassisSpeeds(0,0, angularSpeed);
    ChassisSpeeds speedsRight = new ChassisSpeeds(0,0, -angularSpeed);
    ChassisSpeeds slowLeft = new ChassisSpeeds(0,0, farAngularSpeed);
    ChassisSpeeds slowRight = new ChassisSpeeds(0,0, -farAngularSpeed);
    ChassisSpeeds slowLeft2 = new ChassisSpeeds(0,0, farAngularSpeed/3*2);
    ChassisSpeeds slowRight2 = new ChassisSpeeds(0,0, -farAngularSpeed/3*2);


    ChassisSpeeds stop = new ChassisSpeeds(0,0, 0);
    SwerveRequest.ApplyChassisSpeeds request = new SwerveRequest.ApplyChassisSpeeds();

    
    if(gyro.getYaw().getValueAsDouble() < gyroGoal + threshold && gyro.getYaw().getValueAsDouble() > gyroGoal-threshold){
      TunerConstants.DriveTrain.setControl(request.withSpeeds(stop));
      finished = true;
    }
    else{
      finished = false; // otherwise the true carries over from the last alignment
    }

    //Aligning the robot with the speaker apriltag
    if(gyro.getYaw().getValueAsDouble() > gyroGoal + threshold){
      if(Vision.zshot < 3.5){
        System.out.println("Too far left");
        TunerConstants.DriveTrain.setControl(request.withSpeeds(speedsRight));
      }
      else if(vision.zshot > 3.49 && vision.zshot < 6){
        TunerConstants.DriveTrain.setControl(request.withSpeeds(slowRight));
      }
      else if(vision.zshot > 5.99){
        TunerConstants.DriveTrain.setControl(request.withSpeeds(slowRight2));
      }
    }

    //Aligning the robot with the speaker apriltag
    else if(gyro.getYaw().getValueAsDouble() < gyroGoal - threshold){
      if(Vision.zshot < 3.5){
        System.out.println("Too far right");
        TunerConstants.DriveTrain.setControl(request.withSpeeds(speedsLeft));
      }
      else if(vision.zshot > 3.49 && vision.zshot < 6){
        TunerConstants.DriveTrain.setControl(request.withSpeeds(slowLeft));
      }
      else if(vision.zshot > 5.99){
        TunerConstants.DriveTrain.setControl(request.withSpeeds(slowLeft2));
      }

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
  public void end(boolean interrupted) {
    gyro.close();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}
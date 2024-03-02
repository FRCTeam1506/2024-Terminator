package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import frc.robot.CommandSwerveDrivetrain;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.Swerve;
import frc.robot.generated.TunerConstants;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

  Timer timer = new Timer();
  ShuffleboardTab tab = Shuffleboard.getTab("Vision");

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  NetworkTableEntry tx = table.getEntry("tx");
  NetworkTableEntry ty = table.getEntry("ty");
  NetworkTableEntry tz = table.getEntry("tz");
  NetworkTableEntry ta = table.getEntry("ta");
  NetworkTableEntry tv = table.getEntry("tv");
  NetworkTableEntry botpose = table.getEntry("botpose");
  public static double shooterID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);

  NetworkTable tableIntake = NetworkTableInstance.getDefault().getTable("limelight-front");
  NetworkTableEntry tx2 = tableIntake.getEntry("tx");
  NetworkTableEntry ty2 = tableIntake.getEntry("ty");
  NetworkTableEntry tz2 = tableIntake.getEntry("tz");
  NetworkTableEntry ta2 = tableIntake.getEntry("ta");
  NetworkTableEntry tv2 = tableIntake.getEntry("tv");
  NetworkTableEntry botpose2 = tableIntake.getEntry("botpose");


  public static double x, y, area, target;
  public static double xshot, zshot, shotdist;
  public static double x2, y2, z2, area2, target2;

  private final CommandSwerveDrivetrain m_drivetrain;

  public Vision(CommandSwerveDrivetrain drivetrain) {
    m_drivetrain = drivetrain;
  }

  public void align(){
    // if()
    SwerveRequest.RobotCentric forwardStraight = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
    // };
    TunerConstants.DriveTrain.applyRequest(() -> forwardStraight.withVelocityX(1).withVelocityY(1).withRotationalRate(1));
  }


  public void dashboard(){
    // Field2d m_field = new Field2d();
    // m_field.setRobotPose(estimator.getEstimatedPosition());
    // m_field.setRobotPose(TunerConstants.DriveTrain.getOdometry().getPoseMeters());

    //post to smart dashboard periodically
    // SmartDashboard.putNumber("LimelightX", x);
    // SmartDashboard.putNumber("LimelightY", y);
    // SmartDashboard.putNumber("LimelightShooterDistance", shotdist); // if we want to update shotDist in shuffleboard uncomment this
    // SmartDashboard.putNumber("LimelightArea", area);
    // SmartDashboard.putNumber("Target Exists", target);
    SmartDashboard.putNumber("AprilTag ID", shooterID);

    // SmartDashboard.putNumber("cam x", x);
    // SmartDashboard.putNumber("cam y", y);
    // SmartDashboard.putNumber("cam area", area);
    // SmartDashboard.putNumber("Target exists", target);

    // SmartDashboard.putNumber("pose x", estimator.getEstimatedPosition().getX());
    // SmartDashboard.putNumber("pose y", estimator.getEstimatedPosition().getY());
    // SmartDashboard.putNumber("pose rotation", estimator.getEstimatedPosition().getRotation().getDegrees());
    // SmartDashboard.putString("pose translation", estimator.getEstimatedPosition().getTranslation().toString());

    // SmartDashboard.putData("Field", m_field);


    // tab.addDouble("cam x", () -> x);
    // tab.addDouble("cam y", () -> y);
    // tab.addDouble("cam Area", () -> area);
    // tab.addDouble("Target Exists", () -> target);

    // tab.add(m_field);

    // tab.addDouble("pose x", () -> estimator.getEstimatedPosition().getX());
    // tab.addDouble("pose y", () -> estimator.getEstimatedPosition().getY());
    // tab.addDouble("pose rotation", () -> estimator.getEstimatedPosition().getRotation().getDegrees());
    // tab.addString("pose translation", () -> estimator.getEstimatedPosition().getTranslation().toString());
    
  }

  @Override
  public void periodic() {
    //read values periodically
    dashboard();

    Constants.Swerve.m_field.setRobotPose(m_drivetrain.getState().Pose);

    x = tx.getDouble(0.0);
    y = ty.getDouble(0.0);
    area = ta.getDouble(0.0);
    target = tv.getDouble(0.0);
    shooterID = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0);

    // xshot = Math.abs(LimelightHelpers.getBotPose_TargetSpace("limelight")[0]);
    xshot = 1;
    // zshot = Math.abs(LimelightHelpers.getBotPose_TargetSpace("limelight")[2]);
    zshot = 1;
    // shotdist = Math.sqrt(Math.pow(Math.abs(xshot), 2) + Math.pow(Math.abs(zshot), 2)); //pythagorean theorem

    x2 = tx2.getDouble(0.0);
    y2 = ty2.getDouble(0.0);
    area2 = ta2.getDouble(0.0);
    target2 = tv2.getDouble(0.0);

    if(target == 1){
      m_drivetrain.addVisionMeasurement(LimelightHelpers.getBotPose2d(""), timer.getFPGATimestamp());
    }

  }
}

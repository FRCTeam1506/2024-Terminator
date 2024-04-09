//much of the code is from lasa robotics, Austin TX

package frc.robot.commands.vision;

import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Velocity;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.drivetrain.RotatePIDController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;

public class alignjj extends Command {
  /** Creates a new alignjj. */
  Vision vision;
  double angle;

  SwerveRequest.ApplyChassisSpeeds request = new SwerveRequest.ApplyChassisSpeeds();

  private static final SplineInterpolator SPLINE_INTERPOLATOR = new SplineInterpolator();

  private static final double DRIVE_TURN_INPUT_CURVE_X[] = { 0.0, 0.100, 0.200, 0.300, 0.400, 0.500, 0.600, 0.700, 0.800, 0.900, 1.0 };
  private static final double DRIVE_TURN_INPUT_CURVE_Y[] = { 0.0, 0.010, 0.050, 0.100, 0.150, 0.200, 0.250, 0.300, 0.400, 0.600, 1.0 };
  
  public static final PolynomialSplineFunction DRIVE_TURN_INPUT_CURVE = SPLINE_INTERPOLATOR.interpolate(DRIVE_TURN_INPUT_CURVE_X, DRIVE_TURN_INPUT_CURVE_Y);
  public static final double DRIVE_TURN_SCALAR = 60.0;

  RotatePIDController m_rotatePIDController = new RotatePIDController(DRIVE_TURN_INPUT_CURVE, DRIVE_TURN_SCALAR, 0.1, 6);

/**
 * Align to apriltag for shooting
 * @param angle This should be the intial gyro angle that you want to turn to in terms of the robot
 * 
 */

  public alignjj(Vision vision, double angle) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.vision = vision;
    this.angle = angle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double currentAngle = TunerConstants.DriveTrain.getPigeon2().getAngle();
    double rotateOutput = m_rotatePIDController.calculate(currentAngle, currentAngle + angle);
    System.out.println(rotateOutput);

    ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 3);
    
    // TunerConstants.DriveTrain.applyRequest(() -> request.withSpeeds(speeds));
    // TunerConstants.DriveTrain.runVelocity(speeds);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    // ChassisSpeeds stop = new ChassisSpeeds(0,0, 0);
    // SwerveRequest.ApplyChassisSpeeds request = new SwerveRequest.ApplyChassisSpeeds();
    // TunerConstants.DriveTrain.setControl(request.withSpeeds(stop));

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

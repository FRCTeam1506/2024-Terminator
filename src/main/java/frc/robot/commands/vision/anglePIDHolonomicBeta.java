package frc.robot.commands.vision;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Vision;

/**
 * This command will turn the robot to a specified angle.
 */

//thank you team FRC 5572 github --- https://github.com/Frc5572/FRC2022/blob/main/src/main/java/frc/robot/commands/TurnToAngle.java

public class anglePIDHolonomicBeta extends Command {

    private boolean isRelative;
    private double goal;
    private HolonomicDriveController holonomicDriveController;
    private Pose2d startPos = new Pose2d();
    private Pose2d targetPose2d = new Pose2d();

    SwerveRequest.ApplyChassisSpeeds request = new SwerveRequest.ApplyChassisSpeeds();

    double pX = 10;
    double pY = 100;
    TrapezoidProfile.Constraints constraints = new Constraints(1, 0.5);

    /**
     * Turns robot to specified angle. Uses absolute rotation on field.
     *
     * 
     * @param angle Requested angle to turn to
     * @param isRelative Whether the angle is relative to the current angle: true = relative, false
     *        = absolute
     */

    public anglePIDHolonomicBeta(double angle, boolean isRelative) {
        this.goal = angle;
        this.isRelative = isRelative;

        PIDController xcontroller = new PIDController(pX, 0, 0);
        PIDController ycontroller = new PIDController(pY, 0, 0);
        ProfiledPIDController thetacontroller =
            new ProfiledPIDController(4, 0, 0, constraints);
        holonomicDriveController =
            new HolonomicDriveController(xcontroller, ycontroller, thetacontroller);
        holonomicDriveController.setTolerance(new Pose2d(20, 20, Rotation2d.fromDegrees(1)));

    }

    @Override
    public void initialize() {
        startPos = TunerConstants.DriveTrain.getState().Pose;
        if (isRelative) {
            // targetPose2d = new Pose2d(startPos.getTranslation(),
            //     startPos.getRotation().rotateBy(Rotation2d.fromDegrees(goal)));
            targetPose2d = new Pose2d(0,0,Rotation2d.fromDegrees(TunerConstants.DriveTrain.getPigeon2().getAngle()).rotateBy(Rotation2d.fromDegrees(-goal)));
            System.out.println(targetPose2d.getRotation());
            System.out.println(TunerConstants.DriveTrain.getPigeon2().getAngle() + " gyro angle");
            System.out.println(Vision.x);
        } else {
            targetPose2d = new Pose2d(startPos.getTranslation(), Rotation2d.fromDegrees(goal));
        }
    }

    @Override
    public void execute() {
        // Pose2d currPose2d = TunerConstants.DriveTrain.getState().Pose;
        Pose2d currPose2d = new Pose2d(0, 0, Rotation2d.fromDegrees(TunerConstants.DriveTrain.getPigeon2().getAngle()));
        ChassisSpeeds chassisSpeeds = this.holonomicDriveController.calculate(currPose2d,
            targetPose2d, 0, targetPose2d.getRotation());
        ChassisSpeeds editedSpeeds = new ChassisSpeeds(0,0,chassisSpeeds.omegaRadiansPerSecond);
        TunerConstants.DriveTrain.setControl(request.withSpeeds(editedSpeeds));
    }

    @Override
    public void end(boolean interrupt) {
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0, 0, 0);
        TunerConstants.DriveTrain.setControl(request.withSpeeds(chassisSpeeds));
    }

    @Override
    public boolean isFinished() {
        // return holonomicDriveController.atReference();
        return Math.abs(Vision.x) < 3;

    }
}
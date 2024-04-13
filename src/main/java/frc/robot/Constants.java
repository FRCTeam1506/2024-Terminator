// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;

import frc.robot.generated.TunerConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public final class Constants {

    public static final frc.robot.subsystems.ShooterSubsystem TheShooterSubsystem = new frc.robot.subsystems.ShooterSubsystem();

    public static final class Swerve {        
        static Translation2d fl = new Translation2d(TunerConstants.kFrontLeftXPosInches, TunerConstants.kFrontLeftYPosInches);
        static Translation2d fr = new Translation2d(TunerConstants.kFrontRightXPosInches, TunerConstants.kFrontRightYPosInches);
        static Translation2d rl = new Translation2d(TunerConstants.kBackLeftXPosInches, TunerConstants.kBackLeftYPosInches);
        static Translation2d rr = new Translation2d(TunerConstants.kBackRightXPosInches, TunerConstants.kBackRightYPosInches);

        public static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(fl, fr, rl, rr);
        public static SwerveDrivetrain swerve = TunerConstants.DriveTrain;

        public static double maxLinearVelocity = 4;
        public static double maxAngularVelocity = 4;
        public static double deadband = 0.08;

        public static Field2d m_field = new Field2d();

        // https://www.reddit.com/r/FRC/comments/r6siuy/pid_tuning/

        public static double rotationP = 100 * 2.4; //100
        public static double rotationI = 0; //0
        public static double rotationD = 0.2; //0.2
        public static double driveP = 3*8; //3 //3*8
        public static double driveI = 0; //0
        public static double driveD = 0; //0
        
        // public static double rotationP = 100; //100
        // public static double rotationI = 0; //0
        // public static double rotationD = 12; //0.2
        // public static double driveP = 0.1; //3 //3*8
        // public static double driveI = 0; //0
        // public static double driveD = 0; //0

    }

    public static final class Limelight {
        public static final Double kP = -0.073; // -0.1
        public static final Double MIN_PWR = 0.05;
        public static final Double THRESHOLD = 0.1;
        public static final Double CONVERSION = 0.03355;
        public static final Double LIMIT = 0.4;

        public static final double shooterThreshold = 3;
    }


    public static final class IntakeSubsystem{
        public static final int MOTOR_ID = 51;
        public static final int INDEXER_ID = 52;
        public static final int DIO_PORT = 0;
        public static final double DEFAULT_INTAKE_SPEED = 0.8; //0.3 //0.5
        public static final double DEFAULT_INDEXER_SPEED = 0.2;

        public static final DigitalInput irNine = new DigitalInput(7);
        public static final DigitalInput irEight = new DigitalInput(6);


        public static boolean ring = false;
        public static boolean manualIntake = false;
    }

    public static final class ClimberSubsystem{
        public static final int LEFT_ID = 55;
        public static final int RIGHT_ID = 56;

        public static final double SLOW_DEFAULT_SPEED = 0.15; //0.3
        public static final double NEW_DEFAULT_SPEED = 0.6;
        public static final double CLIMB_POSITION = 5;

        public static boolean endGame = false;
    }


    public static final class CandleSubsystem{
        public static final int CANDLE_ID = 53;
        public static boolean note = false;
        public static boolean noah = false; // noahs ark
    }

    public static final class ShooterSubsystem{
        public static final int TopID = 62;
        public static final int BottomID = 61;
        public static final int AnglerID = 60;

        public static final DigitalInput LimitSwitchDIO = new DigitalInput(3) ;

        public static final double anglerDefaultSpeed = 0.075; //david -- 0.05

        public static final int DRIVE_CONTINUOUS_CL = 35;
        public static final int DRIVE_PEAK_CL       = 60;
        public static final double DRIVE_PEAK_CURRENT_DURATION = 0.1;
        public static final boolean DRIVE_ENABLE_CURRENT_LIMIT = true;

        public static boolean isShooting = false;
        public static boolean autoAim = true;

        //Shooting regression --- ax^2 - bx + c
        public static double a = 0.272199;
        public static double b = 2.74293;
        public static double c = 8.213;

        //this offset should be a negative number if you are shooting too low on red, vice versa
        public static double redOffset = -0.14; //-.14

        //TRAP VALUES
        // static double x = 0.73;
        // public static double topSpeed = 0.4*x;
        // public static double bottomSpeed = .79*x;

        static double x = 0.77;
        public static double topSpeed = 0.3*x;
        public static double bottomSpeed = .8*x+0.1;

        
        public static double anglerSet = 5.1;//75 //5.13
        public static double mailNotesPosition = 4.5;

    }

    public static final class TrapperSubsystem{
        public static final int TRAPPER_VERTICAL_ID = 57;
        public static final int TRAPPER_SHOOTER_ID = 58;
        public static final DigitalInput LimitSwitchDIO = new DigitalInput(8) ;
        public static final double AmpPosition = 8.3; //7.3

    }


    //THANK YOU BRAZILIAN TEAM 1741 -- https://github.com/RAR1741/RA24_RobotCode/blob/columbus/src/main/java/frc/robot/Constants.java 
    public static class Field {
        public static final double k_width = Units.feetToMeters(54.0);
        public static final double k_length = Units.feetToMeters(27.0);

        public static final double k_ampBottom = 26;
        public static final double k_ampTop = 44;

        public static final double k_speakerBottom = 78;
        public static final double k_speakerTop = 82.875;
        public static final double k_speakerAngle = 14.0;

        // TODO: Maybe get these from AprilTags?
        private static final double speakerHeight = 2.032; // Meters
        public static final Pose3d k_redSpeakerPose = new Pose3d(16.579342, 5.547868, speakerHeight, new Rotation3d());
        public static final Pose3d k_blueSpeakerPose = new Pose3d(-0.0381, 5.547868, speakerHeight, new Rotation3d());

        public static final Pose2d k_redPassPose = new Pose2d(14.71, 6.0, new Rotation2d());
        public static final Pose2d k_bluePassPose = new Pose2d(1.75, 6.0, new Rotation2d());

        public static Pose2d getAllianceSpeakerPose2d(){
            if(DriverStation.getAlliance().get() == Alliance.Red){
                return k_redSpeakerPose.toPose2d();
            }
            else{
                return k_blueSpeakerPose.toPose2d();
            }
        }
    }


    public static final Mode currentMode = Mode.REAL;

    public static enum Mode {
      /** Running on a real robot. */
      REAL,
  
      /** Running a physics simulator. */
      SIM,
  
      /** Replaying from a log file. */
      REPLAY
    }


}

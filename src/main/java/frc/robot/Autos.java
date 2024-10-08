// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.autos.BackAndShoot;
import frc.robot.commands.autos.CenterLine;
import frc.robot.commands.autos.ConditionalAutoTest;
import frc.robot.commands.intake.intake;
import frc.robot.commands.shooter.shoot;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

/** Add your docs here. */
public class Autos {

    public static SendableChooser<autos> autoChooser = new SendableChooser<>();

    IntakeSubsystem intake;
    ShooterSubsystem shooter;
    Vision vision;
    Angler angler;
    enum autos { Nothing, TwoMeters, GetThree, Central, GoFar, GoFarDos, MiddleNote, CenterThree, CenterThreeFast,MAD, ThreadTheNeedle, test }


    public Autos(IntakeSubsystem intake, ShooterSubsystem shooter, Angler angler, Vision vision){
        this.intake = intake;
        this.shooter = shooter;
        this.vision = vision;
        this.angler = angler;
        // registerCommands();
        // getAutos();
    }

    public void registerCommands(){
        // NamedCommands.registerCommand("Intake", new intake(intake));
        // NamedCommands.registerCommand("Shoot", new shoot(shooter, intake, angler, vision));
    }

    public void getAutos(){

        autoChooser.setDefaultOption("Nothing", autos.Nothing);
        autoChooser.addOption("TwoMeters", autos.TwoMeters);
        autoChooser.addOption("Shoot and Back", autos.GetThree);
        autoChooser.addOption("Central", autos.Central);
        autoChooser.addOption("Go far", autos.GoFar);
        autoChooser.addOption("Middle Note (+)", autos.MiddleNote);
        // autoChooser.addOption("Go far Dos", autos.GoFarDos);
        autoChooser.addOption("Center Three", autos.CenterThree); 
        autoChooser.addOption("Center Three Fast", autos.CenterThreeFast); 
        autoChooser.addOption("Mutual Asset Denial", autos.MAD);
        autoChooser.addOption("Thread The Needle", autos.ThreadTheNeedle);

        autoChooser.addOption("test", autos.test);

        SmartDashboard.putData(autoChooser);
    }

    public Command sendAutos(){

        switch (autoChooser.getSelected()) {
            case Nothing:
                return new WaitCommand(15.0);
            
            case TwoMeters:
                // return new PathPlannerAuto("Calibration");
                return new PathPlannerAuto("SquareCalibrationAuto");

            case GetThree:
                return new PathPlannerAuto("GetThree");

            case Central:
                return new PathPlannerAuto("Central");

            case MiddleNote:
                return new PathPlannerAuto("MiddleNote");

            case GoFar:
                return new PathPlannerAuto("GoFar");
            
            case GoFarDos:
                // return new PathPlannerAuto("GoFarDos");

            case CenterThree:
            return new PathPlannerAuto("CenterThree");

            case CenterThreeFast:
            return new PathPlannerAuto("CenterThreeBeta");

            case MAD:
            return new PathPlannerAuto("Enraged");

            case ThreadTheNeedle:
            return new PathPlannerAuto("ThreadTheNeedle");

            case test:
            // return new ConditionalAutoTest(shooter, intake, angler, vision, "GoFarOne", null, null, null);
            return new PathPlannerAuto("AmpSide");
                
            default:
                return new WaitCommand(15.0);
        }

    }



}

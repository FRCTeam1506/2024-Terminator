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
    enum autos { Nothing, TwoMeters, GetThree, Central, GoFar, MiddleNote }


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
        SmartDashboard.putData(autoChooser);
    }

    public Command sendAutos(){

        switch (autoChooser.getSelected()) {
            case Nothing:
                return new WaitCommand(15.0);
            
            case TwoMeters:
                // return new PathPlannerAuto("CenterLine");
                // return new PathPlannerAuto("CenterLine");
                return new PathPlannerAuto("Calibration");

            case GetThree:
                // return new PathPlannerAuto("CenterLine");
                return new PathPlannerAuto("GetThree");

            case Central:
                return new PathPlannerAuto("Central");

            case MiddleNote:
                return new PathPlannerAuto("MiddleNote");

            case GoFar:
                // return new PathPlannerAuto("GoFar");
                return new PathPlannerAuto("GoFarDos");
            
            default:
                return new WaitCommand(15.0);
        }

    }



}

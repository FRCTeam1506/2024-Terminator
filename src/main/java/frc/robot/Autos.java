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
import frc.robot.commands.autos.CenterLine;
import frc.robot.commands.intake.intake;
import frc.robot.commands.shooter.shoot;
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
    enum autos { Nothing, CenterLine }


    public Autos(IntakeSubsystem intake, ShooterSubsystem shooter, Angler angler, Vision vision){
        this.intake = intake;
        this.shooter = shooter;
        this.vision = vision;
        this.angler = angler;
        // registerCommands();
        // getAutos();
    }

    public void registerCommands(){
        NamedCommands.registerCommand("Intake", new intake(intake));
        NamedCommands.registerCommand("Shoot", new shoot(shooter, intake, angler, vision));
    }

    public void getAutos(){

        autoChooser.setDefaultOption("Nothing", autos.Nothing);
        autoChooser.addOption("CenterLine", autos.CenterLine);
        SmartDashboard.putData(autoChooser);
    }

    public Command sendAutos(){

        switch (autoChooser.getSelected()) {
            case Nothing:
                return new WaitCommand(15.0);
            
            case CenterLine:
                // return new PathPlannerAuto("CenterLine");
                return new CenterLine(angler, intake, shooter, vision);
            
            default:
                return new WaitCommand(15.0);
        }

    }



}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos.autoShooting;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class stopAnglerIntakeIndexerShooter extends InstantCommand {
  Angler angler;
  IntakeSubsystem intake;
  ShooterSubsystem shooter;
  public stopAnglerIntakeIndexerShooter(Angler angler, IntakeSubsystem intake, ShooterSubsystem shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angler = angler;
    this.intake = intake;
    this.shooter = shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    angler.stopAngler();
    intake.stopIntake();
    intake.stopIndexer();
    shooter.shootStop();
  }
}

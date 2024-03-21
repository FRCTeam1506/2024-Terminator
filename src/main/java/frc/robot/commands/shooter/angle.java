// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterSubsystem;
import frc.robot.subsystems.Angler;

public class angle extends Command {
  /** Creates a new moveAngler. */
  Angler angler;
  boolean finished = false;

  public angle(Angler angler) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angler = angler;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {    
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    angler.setPosition();
    //angler.setPositionByPose();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // angler.stopAngler();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}

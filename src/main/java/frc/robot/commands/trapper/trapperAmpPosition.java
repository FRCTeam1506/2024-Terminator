// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trapper;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trapper;

public class trapperAmpPosition extends Command {
  /** Creates a new trapperAmpPosition. */
  Trapper trapper;
  public trapperAmpPosition(Trapper trapper) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.trapper = trapper;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    trapper.setAmpPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    trapper.stopTrapper();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

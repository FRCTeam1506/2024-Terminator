// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterSubsystem;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.Vision;

public class angleDistance extends Command {
  /** Creates a new moveAngler. */
  Angler angler;
  double distance;
  boolean finished = false;

  public angleDistance(Angler angler, double distance) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.angler = angler;
    this.distance = distance;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double angle = Math.toDegrees(Math.atan(66/(distance * 39.37)))/5.14; 
    angler.setPosition();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    angler.stopAngler();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}

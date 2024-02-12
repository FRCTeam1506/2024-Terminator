// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterSubsystem;

public class moveAngler extends Command {
  /** Creates a new moveAngler. */
  frc.robot.subsystems.ShooterSubsystem shooter;
  double moe = .15;
  double setpoint;
  boolean finished = false;

  public moveAngler(frc.robot.subsystems.ShooterSubsystem shooter, double setpoint) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = shooter;
    this.setpoint = setpoint;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (shooter.getAngle() > setpoint - moe && shooter.getAngle() < setpoint + moe) {
      shooter.anglerStop();
      finished = true;
    }
    else if(shooter.getAngle() > setpoint + moe){
      shooter.anglerDown();
      finished = false;
    }
    else if (shooter.getAngle() < setpoint - moe){
      shooter.anglerUp();
      finished = false;
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished;
  }
}

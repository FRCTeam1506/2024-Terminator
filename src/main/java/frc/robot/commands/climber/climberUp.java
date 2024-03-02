// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climber;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Climber;

public class climberUp extends Command {
  /** Creates a new climberUp. */
  Climber climber;
  public climberUp(Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    System.out.println(climber.getAvgPosition());
    if(climber.getAvgPosition() < 60){
      climber.set(0.6);
    }
    else if(climber.getAvgPosition() < 120){
      climber.set(0.3);
    }
    else{
      climber.set(0.1);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

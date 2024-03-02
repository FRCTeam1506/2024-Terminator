// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.intake;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeSubsystem;

public class climberIntakeLuke extends Command {
  /** Creates a new runWheel. */
  IntakeSubsystem intake;
  Climber climber;
  public climberIntakeLuke(IntakeSubsystem intake, Climber climber) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.intake = intake;
    this.climber = climber;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Constants.ClimberSubsystem.endGame){
      climber.setRightClimber(Constants.ClimberSubsystem.NEW_DEFAULT_SPEED);
    }
    else if (!Constants.ClimberSubsystem.endGame){
      intake.intake();
    }
    
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Constants.IntakeSubsystem.ring;
  }
}

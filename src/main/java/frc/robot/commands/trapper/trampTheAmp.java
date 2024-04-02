// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.trapper;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Trapper;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class trampTheAmp extends SequentialCommandGroup {
  /** Creates a new trampTheAmp. */
  public trampTheAmp(Trapper trapper, ShooterSubsystem shooter, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new trapperAmpPosition(trapper).withTimeout(0.3), 
      new ParallelDeadlineGroup(
        new WaitCommand(1),
        new trapperAmpPosition(trapper),
        new WaitCommand(.5),
        new InstantCommand(() -> trapper.intake())),
      new WaitCommand(3)
    );
  }
}

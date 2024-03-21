// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.angler.setPosition;
import frc.robot.commands.intake.intake;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class prepareToShoot extends SequentialCommandGroup {
  /** Creates a new prepareToShoot. */
  public prepareToShoot(Angler angler, ShooterSubsystem shooter, IntakeSubsystem intake, double position) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new setPosition(angler, position),
        new runWheel(shooter),
        new intake(intake))
    );
  }
}

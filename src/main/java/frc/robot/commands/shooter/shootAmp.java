// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.angler.ampPosition;
import frc.robot.commands.intake.indexToShoot;
import frc.robot.commands.intake.runIndexer;
import frc.robot.commands.intake.stopIndexer;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.ShooterSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootAmp extends SequentialCommandGroup {
  /** Creates a new shootAmp. */
  public shootAmp(Angler angler, ShooterSubsystem shooter, IntakeSubsystem intake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new ampPosition(angler),
        new shootAmpPower(shooter)
      ).withTimeout(0.6),
      new WaitCommand(0.2),
      new indexToShoot(intake).withTimeout(0.2),
      new stopShooter(shooter).withTimeout(0.04),
      new stopIndexer(intake).withTimeout(0.1)
    );
  }
}

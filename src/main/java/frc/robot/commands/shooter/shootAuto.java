// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

import frc.robot.Constants;
import frc.robot.commands.angler.setPosition;
import frc.robot.commands.angler.stopAngler;
import frc.robot.commands.intake.runIndexer;
import frc.robot.commands.intake.stopIndexer;
import frc.robot.commands.vision.vision2;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootAuto extends SequentialCommandGroup {
  /** Creates a new shoot. */
  public shootAuto(ShooterSubsystem shooter, IntakeSubsystem intake, Angler angler, Vision vision, double setpoint) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        new setPosition(angler, setpoint),//.until(() -> angler.getPos() > angler.getVisionPosition()),
        new runWheel(shooter).withTimeout(0.1)
        // new angle(angler, Math.toDegrees(Math.atan(66/(z * 39.37)))/5.14).until(() -> angler.getPos() > Math.toDegrees(Math.atan(66/(z * 39.37)))/5.14)
      ).withTimeout(0.6),
      //new stopShooter(shooter),
      new stopAngler(angler).withTimeout(0.1),
      //new WaitCommand(0.5),
      new runIndexer(intake).withTimeout(0.3),
      new ParallelCommandGroup(
        new stopIndexer(intake),
        new stopShooter(shooter)
      ).withTimeout(0.1)
    );
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

import frc.robot.Constants;
import frc.robot.commands.angler.setPosition;
import frc.robot.commands.angler.stopAngler;
import frc.robot.commands.drivetrain.brake;
import frc.robot.commands.drivetrain.stop;
import frc.robot.commands.intake.runIndexer;
import frc.robot.commands.intake.stopIndexer;
import frc.robot.commands.vision.vision2;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PREmailNotes extends SequentialCommandGroup {
  /** Creates a new shoot. */
  public PREmailNotes(ShooterSubsystem shooter, IntakeSubsystem intake, Angler angler, Vision vision, double anglerSetpoint, double shooterPower) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    Constants.ShooterSubsystem.isShooting = true;
    addCommands(
      new ParallelCommandGroup(
        new setPosition(angler, anglerSetpoint),
        new runWheelPower(shooter, shooterPower)
      ).withTimeout(0.5)
    );
    Constants.ShooterSubsystem.isShooting = false;
  }
}

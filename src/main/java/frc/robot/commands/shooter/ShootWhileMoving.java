// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;

import frc.robot.Constants;
import frc.robot.commands.angler.stopAngler;
import frc.robot.commands.drivetrain.brake;
import frc.robot.commands.drivetrain.stop;
import frc.robot.commands.intake.indexToShoot;
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
public class ShootWhileMoving extends SequentialCommandGroup {
  /** Creates a new shoot. */

  /**
   * Will continually move the angler to the correct angle, waiting to shoot until the time has elapsed
   *  @param time Length of time to wait to shoot (seconds)
   */

  public ShootWhileMoving(ShooterSubsystem shooter, IntakeSubsystem intake, Angler angler, Vision vision, double time) {
    addCommands(
      new runWheel(shooter).withTimeout(0.05),
      new ParallelCommandGroup(
        new stopAngler(angler).withTimeout(time-0.5).andThen(new RepeatCommand(new angleForMovingShot(angler))), //timeout in there bc it was going too high to start and having trouble coming back down
        new stopIndexer(intake).withTimeout(time).andThen(new indexToShoot(intake))
      ).withTimeout(time + 0.4),
      new stopShooter(shooter).withTimeout(0.01),
      new stopIndexer(intake).withTimeout(0.01)
    );
  }
}

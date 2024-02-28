// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.intake;
import frc.robot.commands.shooter.shoot;
import frc.robot.commands.shooter.shootAuto;
import frc.robot.commands.shooter.shootNoAngle;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BackAndShoot extends SequentialCommandGroup {
  /** Creates a new CenterLine. */
  public BackAndShoot(Angler angle, IntakeSubsystem intake, ShooterSubsystem shooter, Vision vision) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new shootNoAngle(shooter, intake, angle, vision),
      // new intake(intake).withTimeout(0.1),
      TunerConstants.DriveTrain.followPathCommand("Backward")
    );
  }
}

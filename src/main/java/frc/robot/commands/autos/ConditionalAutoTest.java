// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Angler;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.Vision;
import frc.robot.commands.autos.autoShooting.ShootBase;
import frc.robot.commands.drivetrain.zeroGyro;
import frc.robot.commands.intake.stopIntake;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ConditionalAutoTest extends SequentialCommandGroup {
  /** Creates a new ConditionalAutoTest. */
  public ConditionalAutoTest(ShooterSubsystem shooter, IntakeSubsystem intake, Angler angler, Vision vision, String one, String two, String three, String four) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    PathPlannerPath path1 = PathPlannerPath.fromPathFile(one);
    // PathPlannerPath path2 = PathPlannerPath.fromPathFile(two);
    // PathPlannerPath path3 = PathPlannerPath.fromPathFile(three);
    // PathPlannerPath path4 = PathPlannerPath.fromPathFile(four);

    addCommands(
      new zeroGyro().withTimeout(0.1),
      new ShootBase(shooter, intake, angler, vision),
      new ParallelDeadlineGroup(
        AutoBuilder.followPath(path1),
        new frc.robot.commands.intake.intake(intake)
      ),
      new WaitCommand(0.2),
      new stopIntake(intake).withTimeout(0.1)    
    );
  }
}

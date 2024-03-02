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
import frc.robot.subsystems.Trapper;
import frc.robot.subsystems.Vision;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class shootConditional extends SequentialCommandGroup {
  /** Creates a new shoot. */
  public shootConditional(ShooterSubsystem shooter, IntakeSubsystem intake, Angler angler, Vision vision, Trapper trapper) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    double z = vision.zshot;
    Constants.ShooterSubsystem.isShooting = true;

    if(!Constants.ClimberSubsystem.endGame){
      if(Constants.ShooterSubsystem.autoAim){
        addCommands(
          new ParallelDeadlineGroup(        
            new vision2(vision).until(() -> vision.x > -Constants.Limelight.shooterThreshold && vision.x < Constants.Limelight.shooterThreshold),
            new runWheel(shooter).withTimeout(0.1)
          ),
          new stop().withTimeout(0.05),
          new ParallelDeadlineGroup(
            new angle(angler),//.until(() -> angler.getPos() > angler.getVisionPosition()),
            new brake().withTimeout(0.2)
          ).withTimeout(0.4),
          //new stopShooter(shooter),
          // new stopAngler(angler),
          //new WaitCommand(0.5),
          // new ParallelCommandGroup(
          //   new runWheel(shooter),
          //   new brake()
          // ).withTimeout(0.2),
          new runIndexer(intake),
          new WaitCommand(0.1),
          new stopIndexer(intake)
        );
      }
      else if(!Constants.ShooterSubsystem.autoAim){
        addCommands(        
          new ParallelDeadlineGroup(
            new angle(angler),//.until(() -> angler.getPos() > angler.getVisionPosition()),
            new runWheel(shooter).withTimeout(0.1),
            new brake().withTimeout(0.2)
          ).withTimeout(0.8),
          //new stopShooter(shooter),
          // new stopAngler(angler),
          //new WaitCommand(0.5),
          new ParallelCommandGroup(
            new runWheel(shooter),
            new brake()
          ).withTimeout(0.2),
          new runIndexer(intake),
          new WaitCommand(0.1),
          new stopIndexer(intake)
        );
      }
      Constants.ShooterSubsystem.isShooting = false;
    }
    else if(Constants.ClimberSubsystem.endGame){
      trapper.intake();
    }
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Throat;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Throat;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class startThroat extends Command {
  public SS_Throat throat;

  /** Creates a new startThroat. */
  public startThroat(SS_Throat ss_throat) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_throat);
    this.throat = ss_throat;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    throat.timer.stop();
    throat.timer.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    throat.timer.start();
    if (throat.timer.hasElapsed(0.8)) {
      throat.throatMotor.set(0.7);
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    throat.timer.stop();
    throat.timer.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.TurretShooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class manualShooter extends Command {
  public SS_Shooter shooter;
  /** Creates a new manualShooter. */
  public manualShooter(SS_Shooter ss_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_shooter);
    this.shooter = ss_shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    shooter.leftShooter.set(-1);
    shooter.rightShooter.set(1);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

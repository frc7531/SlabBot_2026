// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Shooter;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class moveActuator extends Command {
  /** Creates a new moveActuator. */
  public SS_Shooter shooter;
  public moveActuator(SS_Shooter ss_shooter) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.shooter = ss_shooter;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //System.out.println("- .... .. ... / ... .... --- ..- .-.. -.. / .-- --- .-. -.-");
    shooter.leftHoodLifter.setPosition(shooter.actuatorPosition.getDouble(0));
    shooter.rightHoodLifter.setPosition(shooter.actuatorPosition.getDouble(0));
    //System.out.println("Running");
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

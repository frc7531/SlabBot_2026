// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SS_Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class flipTurret extends Command {
  /** Creates a new flipTurret. */

  public SS_Turret turret;
  public PIDController tController = new PIDController(0.1, 0, 0);
  public double tSpeed;
  public CANcoder encoder;

  public flipTurret(SS_Turret ss_turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_turret);
    this.turret = ss_turret;
    this.encoder = ss_turret.encoder;
    withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    tController.reset();
    tController.setTolerance(0.5);
  }

  public Command withTarget(int direction, double offset) {
    tController.setSetpoint(0.5*direction + 2*offset);
    return this;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    tSpeed = tController.calculate(turret.getTurretRotation());
    // turret.setRawSpeed(tSpeed);
    System.out.println("speed: " + tSpeed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return tController.atSetpoint();
  }
}

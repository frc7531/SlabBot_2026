// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.vision.flipTurret;

public class SS_Turret extends SubsystemBase {
  public Encoder encoder;
  public TalonFX turretMotor;
  public flipTurret flippy;

  /** Creates a new SS_Turret. */
  public SS_Turret() {
    encoder.reset();
    encoder.setDistancePerPulse(360/2048);
    flippy = new flipTurret(this);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void setSpeed(double speed) {
    if (Math.abs(encoder.getDistance() + speed) > 180) {
      CommandScheduler.getInstance().schedule(flippy.withTarget((int) -Math.signum(encoder.getDistance()), speed));
    } else {
      turretMotor.set(speed);
    }
  }

  public void setRawSpeed(double speed) {
    turretMotor.set(speed);
  }
}

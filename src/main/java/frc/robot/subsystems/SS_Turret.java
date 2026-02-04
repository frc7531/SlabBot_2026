// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.vision.flipTurret;

public class SS_Turret extends SubsystemBase {
  public CANcoder encoder = new CANcoder(43);
  public TalonFX turretMotor = new TalonFX(40);
  public Pigeon2 pidgey = new Pigeon2(50);
  public flipTurret flippy;
  public final double leftMaximum = 0.4;
  public final double rightMaximum = -0.15;

  /** Creates a new SS_Turret. */
  public SS_Turret() {
    encoder.setPosition(0);
    turretMotor.setNeutralMode(NeutralModeValue.Brake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    System.out.println(getTurretRotation());
  }

  // All of these inputs should be relative to the turret, not the motor that drives it

  public double getTurretRotation() {
    SmartDashboard.putNumber("TurretRotation", -0.1*encoder.getPosition().getValueAsDouble());
    return -0.1*encoder.getPosition().getValueAsDouble();
  }

  public Translation2d getTurretPosition() {
    Translation2d position = new Translation2d(0.2032*Math.cos(pidgey.getYaw().getValueAsDouble()), 0.2032*Math.sin(pidgey.getYaw().getValueAsDouble()));
    return position;
  }

  public void setTurretSpeed(double speed) {
    if (Math.abs(getTurretRotation()) > 0.2) { // If the attempted move would probably twist up the turret
      // flippy.withTarget((int) -Math.signum(encoder.getPosition().getValueAsDouble()), speed);
      // // if (Math.abs(getTurretRotation() + speed) - 1 > -0.49) { // If flipping can actually help
      //   CommandScheduler.getInstance().schedule(flippy);
      // } else {
      //   System.out.println("Can't reach the angle !!!");
      // }
    } else {
      turretMotor.set(speed);
    }
  }

  public void setRawSpeed(double speed) {
    turretMotor.set(speed);
  }
}

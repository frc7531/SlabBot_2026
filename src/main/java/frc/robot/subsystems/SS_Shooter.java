// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.PWM;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Shooter extends SubsystemBase {
  public final double shooterMaxSpeed = 17.5; //Theoretical, not gonna be exact yet

  // Constants stuff all in meters -ish
  public double lipHeight = 1.8288;
  public double hoopRadius = 0.5969;
  public double gravity = 4.90335;
  public double linearActuatorAngle = 60;
  public double hoodArmRadius = 0.15875;
  public double heightDifference = 0.0762;
  public double maxExtension = 0.1397;

  // Set these to change the trajectory
  public double targetHeight = 1.8;
  public double lipClearance = 0.2;

  // Precalculated stuff
  public double clearanceHeight = lipHeight + lipClearance;

  // Stuff to be calculated
  public double lipDistance;
  public double targetVelocity;
  public double targetAngle;
  public double targetPosition;

  // Hard goods
  public PWM leftHoodLifter = new PWM(0);
  public PWM rightHoodLifter = new PWM(1);
  public TalonFX leftShooter = new TalonFX(42);
  public TalonFX rightShooter = new TalonFX(41);

  public NetworkTableInstance inst = NetworkTableInstance.getDefault();
  public NetworkTableEntry actuatorPosition = inst.getTable("Shooter").getEntry("Actuator Position");

  /** Creates a new SS_Shooter. */
  public SS_Shooter() {
    actuatorPosition.setDouble(0.0);
    leftShooter.setNeutralMode(NeutralModeValue.Coast);
    rightShooter.setNeutralMode(NeutralModeValue.Coast);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    inst.getTable("Shooter").getEntry("Actuator Position");
  }

  public double calculateGoalAngle(double distance) {
    lipDistance = distance - hoopRadius;
    targetAngle = Math.atan((distance*clearanceHeight)/(lipDistance*hoopRadius) - (targetHeight*lipDistance)/(distance*hoopRadius));
    return targetAngle;
  }

  public void setVelocity(double distance, double shooterAngle) {
    lipDistance = distance - hoopRadius;
    targetVelocity = Math.sqrt(gravity/(clearanceHeight/(hoopRadius*lipDistance) - targetHeight/(distance*hoopRadius)))/Math.cos(shooterAngle);
    leftShooter.set(targetVelocity/shooterMaxSpeed);
    rightShooter.set(targetVelocity/shooterMaxSpeed);
  }

  public void setHoodAngle(double angle) {
    targetPosition = ((hoodArmRadius*Math.sin(angle) - heightDifference)/Math.sin(linearActuatorAngle))/maxExtension;
    if ((targetPosition < 0.01) || (targetPosition > 0.99)) {
      System.out.println("Clamping Hood Angle!!!");
      leftHoodLifter.setPosition(MathUtil.clamp(targetPosition, 0.01, 0.99));
      rightHoodLifter.setPosition(MathUtil.clamp(targetPosition, 0.01, 0.99));
    } else {
      leftHoodLifter.setPosition(targetPosition);
      rightHoodLifter.setPosition(targetPosition);
    }
  }
}

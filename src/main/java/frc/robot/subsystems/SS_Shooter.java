// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Shooter extends SubsystemBase {
  // Constants stuff
  public double lipHeight = 1.8288;
  public double hoopRadius = 0.5969;
  public double gravity = 4.90335;

  // Set these to change the trajectory
  public double targetHeight = 1.8;
  public double lipClearance = 0.2;

  // Precalculated stuff
  public double clearanceHeight = lipHeight + lipClearance;

  // Stuff to be calculated
  public double lipDistance;
  public double targetVelocity;
  public double targetAngle;
   
  /** Creates a new SS_Shooter. */
  public SS_Shooter() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public double calculateGoalAngle(double distance) {
    lipDistance = distance - hoopRadius;
    targetAngle = Math.atan((distance*clearanceHeight)/(lipDistance*hoopRadius) - (targetHeight*lipDistance)/(distance*hoopRadius));
    return targetAngle;
  }

  public void setVelocity(double distance, double shooterAngle) {
    lipDistance = distance - hoopRadius;
    targetVelocity = Math.sqrt(gravity/(clearanceHeight/(hoopRadius*lipDistance) - targetHeight/(distance*hoopRadius)))/Math.cos(shooterAngle);
  }
}

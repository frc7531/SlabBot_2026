// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Vision extends SubsystemBase {
  /** Creates a new SS_Vision. */

  NetworkTable limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

  double[] poseEstimate;

  public SS_Vision() {
    poseEstimate = limelightTable.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[6]);
  }

  public void setVariables() {
    limelightTable = NetworkTableInstance.getDefault().getTable("limelight");

    if (limelightTable != null) {
      poseEstimate = limelightTable.getEntry("botpose_orb_wpiblue").getDoubleArray(poseEstimate);
    }
  }

  public double[] getPoseEstimate() {
    return poseEstimate;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

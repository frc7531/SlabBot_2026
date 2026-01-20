// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SS_Vision extends SubsystemBase {
  /** Creates a new SS_Vision. */

  NetworkTable limelightTableAntigua = NetworkTableInstance.getDefault().getTable("limelight-antigua");
  NetworkTable limelightTableBarbuda = NetworkTableInstance.getDefault().getTable("limelight-barbuda");

  double[] poseEstimateAntigua;
  double[] poseEstimateBarbuda;

  public SS_Vision() {
    poseEstimateAntigua = limelightTableAntigua.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[6]);
    poseEstimateBarbuda = limelightTableBarbuda.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[6]);
  }

  public void setVariables() {
    limelightTableAntigua = NetworkTableInstance.getDefault().getTable("limelight-antigua");
    limelightTableBarbuda = NetworkTableInstance.getDefault().getTable("limelight-barbuda");

    if (limelightTableAntigua != null) {
      poseEstimateAntigua = limelightTableAntigua.getEntry("botpose_orb_wpiblue").getDoubleArray(poseEstimateAntigua);
    }
    if (limelightTableBarbuda != null) {
      poseEstimateBarbuda = limelightTableBarbuda.getEntry("botpose_orb_wpiblue").getDoubleArray(poseEstimateBarbuda);
    }
  }

  public double[] getPoseEstimateAntigua() {
    return poseEstimateAntigua;
  }

  public double[] getPoseEstimateBarbuda() {
    return poseEstimateBarbuda;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

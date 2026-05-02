// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.util.struct.Struct;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.LimelightHelpers.PoseEstimate;

public class SS_Vision extends SubsystemBase {

  private final Pigeon2 pidgey = new Pigeon2(0);

  NetworkTable limelightTableAntigua = NetworkTableInstance.getDefault().getTable("limelight-antigua");
  NetworkTable limelightTableBarbuda = NetworkTableInstance.getDefault().getTable("limelight-barbuda");

  double[] poseEstimateAntigua;
  double[] poseEstimateBarbuda;

  public DriverStation.Alliance alliance;

  // Field Poses
  public Translation2d hubPose = new Translation2d(4.625594, 4.034536);
  public Translation2d towerPose = new Translation2d(1.7113, 4.134044);
  public Translation2d depotPose = new Translation2d(0.5, 6.391656);
  public Translation2d stationPose = new Translation2d(0.5, 1.651);

  double orientationShift = 0;
  double stdDev;

  SS_Drivetrain drivetrain;

  public SS_Vision() {
    poseEstimateAntigua = limelightTableAntigua.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[6]);
    poseEstimateBarbuda = limelightTableBarbuda.getEntry("botpose_orb_wpiblue").getDoubleArray(new double[6]);
  }

  @Override
  public void periodic() {
    addVisionMeasurement(updateVisionPosition("limelight-antiqua"));
    addVisionMeasurement(updateVisionPosition("limelight-barbuda"));
  }

  private PoseEstimate updateVisionPosition(String limelightName) {
    switch (alliance) {
      case Blue:
        orientationShift = 0;
        break;
      case Red:
        orientationShift = 180;
        break;
    }
    LimelightHelpers.SetRobotOrientation(limelightName,
        pidgey.getRotation2d().getDegrees() + orientationShift,
        pidgey.getAngularVelocityZDevice().getValueAsDouble(), 0, 0, 0, 0);
    LimelightHelpers.PoseEstimate mt2 = null;
    switch (alliance) {
      case Blue:
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2(limelightName);
        break;
      case Red:
        mt2 = LimelightHelpers.getBotPoseEstimate_wpiRed_MegaTag2(limelightName);
        break;
    }
    if (mt2 == null || mt2.tagCount == 0 || Math.abs(pidgey.getAngularVelocityZDevice().getValueAsDouble()) > 720)
      return null;
    else
      return mt2; // Reject during fast rotation
  }

  private void addVisionMeasurement(PoseEstimate mt2) {
    stdDev = MathUtil.clamp(1.1 * mt2.avgTagDist / mt2.tagCount, 0.35, 1.1);
    drivetrain.poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(stdDev, stdDev, 0.5));
    // Use the vision translation but prefer the Pigeon/IMU for robot heading.
    // Vision headings can be noisy or use a different convention and were
    // overwriting the estimator's heading, causing the rotated localization.
    Pose2d visionPoseUsingGyroHeading = new Pose2d(mt2.pose.getTranslation(), pidgey.getRotation2d());
    drivetrain.poseEstimator.addVisionMeasurement(visionPoseUsingGyroHeading, mt2.timestampSeconds);
  }

  public double[] getPoseEstimateAntigua() {
    return poseEstimateAntigua;
  }

  public double[] getPoseEstimateBarbuda() {
    return poseEstimateBarbuda;
  }
}

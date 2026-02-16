// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.Meters;

import java.util.spi.CurrencyNameProvider;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.SS_Drivetrain;
import frc.robot.subsystems.SS_Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class alignTower extends Command {

  public SS_Vision vision;
  public RobotContainer robotContainer;
  public SS_Drivetrain drivetrain;
  SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric();
  NetworkTable limelightTableBarbuda = NetworkTableInstance.getDefault().getTable("limelight-barbuda");
  Translation2d estimatedPose;

  PIDController rController = new PIDController(0.08, 0, 0);
  PIDController xController = new PIDController(1.6, 0, 0);
  PIDController yController = new PIDController(1.6, 0, 0);

  Translation2d targetPose;
  Translation2d poseDifference;
  Double targetXPoseDifference;
  Double targetYPoseDifference;
  private double targetAngle;

  double currentAngle;

  double rSpeed;
  double xSpeed;
  double ySpeed;

  /** Creates a new alignTower. */
  public alignTower(SS_Vision ss_vision, SS_Drivetrain ss_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_vision, ss_drivetrain);
    this.vision = ss_vision;
    this.drivetrain = ss_drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rController.setSetpoint(180);
    rController.setTolerance(0.01);

    xController.setSetpoint(0);
    xController.setTolerance(0.01);

    yController.setSetpoint(0);
    yController.setTolerance(0.01);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetPose = drivetrain.towerPose;

    estimatedPose = drivetrain.poseEstimator.getEstimatedPosition().getTranslation();
    poseDifference = targetPose.minus(estimatedPose);

    currentAngle = drivetrain.pidgey.getRotation2d().getDegrees();
    targetXPoseDifference = poseDifference.getMeasureX().in(Meters);
    targetYPoseDifference = poseDifference.getMeasureY().in(Meters);

    // if (currentAngle > 180) {
    //   targetAngle = 180 - (currentAngle % 180);
    // } else if (currentAngle < -180) {
    //   targetAngle = 180 + (currentAngle % 180);
    // } else {
    //   targetAngle = currentAngle;
    // }

    targetAngle = Math.floorMod((int) currentAngle, 360);

    rSpeed = rController.calculate(targetAngle);
    xSpeed = xController.calculate(targetXPoseDifference);
    ySpeed = yController.calculate(targetYPoseDifference);

    SmartDashboard.putNumber("rSpeed", rSpeed);
    SmartDashboard.putNumber("xSpeed", xSpeed);
    SmartDashboard.putNumber("ySpeed", ySpeed);
    SmartDashboard.putNumber("ccccurrentAngle", currentAngle);
    SmartDashboard.putNumber("ttttargetAngle", targetAngle);

    drivetrain.setControl(drive.withVelocityX(xSpeed).withVelocityY(ySpeed).withRotationalRate(rSpeed));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SS_Drivetrain;
import frc.robot.subsystems.SS_Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class aimTurretToTarget extends Command {
  /** Creates a new aimTurretToTarget. */
  public SS_Drivetrain drivetrain;
  public SS_Turret turret;

  PIDController rController = new PIDController(3.2, 0.2, 0.002); //0.8
  double pidSpeed;

  Translation2d estimatedPose;

  Translation2d targetPose;
  double targetAngle;

  double turretRotations;
  double turretAngle;
  Translation2d turretEstimate;

  Translation2d poseDifference;
  double rotationsDifference;

  double newAngle;

  public SwerveRequest.RobotCentric driverequest = new SwerveRequest.RobotCentric();
  public CommandXboxController controller;
  public CANcoder encoder;

  public aimTurretToTarget(SS_Drivetrain ss_drivetrain, SS_Turret ss_turret) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_turret);
    //this.controller = joystick;
    this.drivetrain = ss_drivetrain;
    this.turret = ss_turret;
    this.encoder = ss_turret.encoder;
    //withInterruptBehavior(InterruptionBehavior.kCancelSelf);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    rController.reset();
    rController.setSetpoint(0);
    rController.setTolerance(0.5);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetPose = drivetrain.hubPose;

    estimatedPose = drivetrain.poseEstimator.getEstimatedPosition().getTranslation();
    turretEstimate = estimatedPose.plus(turret.getTurretPosition());

    poseDifference = targetPose.minus(turretEstimate);
    targetAngle = poseDifference.getAngle().getDegrees();

    turretRotations = turret.getTurretRotation();
    turretAngle = 360*turretRotations + drivetrain.poseEstimator.getEstimatedPosition().getRotation().getDegrees();
    rotationsDifference = (turretAngle - targetAngle)/360;

    if ((targetAngle + drivetrain.pidgey.getYaw().getValueAsDouble() > 360*turret.leftMaximum) || (targetAngle + drivetrain.pidgey.getYaw().getValueAsDouble() < 360*turret.rightMaximum)) {
      if (turretAngle >= 0) {
        if (targetAngle >= 0) {
          newAngle = targetAngle - 360;
        } else {
          newAngle = targetAngle;
        }

        if (-newAngle < 360*turret.rightMaximum) {
          rotationsDifference = (turretAngle - 20)/360;
        }
      } else {
        if (targetAngle < 0) {
          newAngle = targetAngle + 360;
        } else {
          newAngle = targetAngle;
        }

        if (newAngle < 360*turret.leftMaximum) {
          rotationsDifference = (turretAngle + 20)/360;
        }
      }
    }

    //pidSpeed = rController.calculate(rotationsDifference);

    //turret.setRawSpeed(pidSpeed); //This is the turret's speed
    SmartDashboard.putNumber("targetAngle", targetAngle);
    SmartDashboard.putNumber("turretAngle", turretAngle);
    SmartDashboard.putNumber("newAngle", newAngle);
    SmartDashboard.putNumber("rotationsDifference", rotationsDifference);
    //System.out.println("speed: " + pidSpeed);
    turret.setRawSpeed(0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; // This is a default, so leaving this as false
  }
}

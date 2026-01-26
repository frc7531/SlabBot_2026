// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SS_Drivetrain;
import frc.robot.subsystems.SS_Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class aimTurretToTarget extends Command {
  /** Creates a new findTarget. */
  public SS_Drivetrain drivetrain;
  public SS_Turret turret;

  PIDController rController = new PIDController(0.1, 0, 0);
  Translation2d estimatedPose;
  Translation2d blueHubPose = new Translation2d(4.625594, 4.034536);
  Translation2d redHubPose = new Translation2d(4.625594, 4.034536);
  Translation2d targetPose;
  Translation2d poseDifference;
  Rotation2d targetAngle;
  Rotation2d currentAngle;
  Rotation2d angleDifference;
  Rotation2d angleShift;
  double pidSpeed;
  public SwerveRequest.RobotCentric driverequest = new SwerveRequest.RobotCentric();
  public CommandXboxController controller;

  public aimTurretToTarget(SS_Drivetrain ss_drivetrain, SS_Turret ss_turret, CommandXboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_turret);
    this.controller = joystick;
    this.drivetrain = ss_drivetrain;
    this.turret = ss_turret;
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
    switch (drivetrain.alliance) {
      case Red:
        targetPose = redHubPose;
      case Blue:
        targetPose = blueHubPose;
    }
    estimatedPose = drivetrain.poseEstimator.getEstimatedPosition().getTranslation();

    poseDifference = targetPose.minus(estimatedPose);
    targetAngle = poseDifference.getAngle();
    currentAngle = drivetrain.pidgey.getRotation2d();
    angleDifference = currentAngle.minus(targetAngle);
    pidSpeed = rController.calculate(angleDifference.getDegrees());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

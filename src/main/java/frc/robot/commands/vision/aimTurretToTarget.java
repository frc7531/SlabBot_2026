// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.SS_Drivetrain;
import frc.robot.subsystems.SS_Turret;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class aimTurretToTarget extends Command {
  /** Creates a new aimTurretToTarget. */
  public SS_Drivetrain drivetrain;
  public SS_Turret turret;

  PIDController rController = new PIDController(0.1, 0, 0);
  double pidSpeed;

  Translation2d estimatedPose;

  Translation2d blueHubPose = new Translation2d(4.625594, 4.034536);
  Translation2d redHubPose = new Translation2d(4.625594, 4.034536);

  Translation2d targetPose;
  double targetAngle;

  double turretAngle;

  Translation2d poseDifference;
  double angleDifference;

  public SwerveRequest.RobotCentric driverequest = new SwerveRequest.RobotCentric();
  public CommandXboxController controller;
  public Encoder encoder;

  public aimTurretToTarget(SS_Drivetrain ss_drivetrain, SS_Turret ss_turret, CommandXboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_turret);
    this.controller = joystick;
    this.drivetrain = ss_drivetrain;
    this.turret = ss_turret;
    this.encoder = ss_turret.encoder;
    withInterruptBehavior(InterruptionBehavior.kCancelSelf);
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
    targetAngle = poseDifference.getAngle().getDegrees();

    turretAngle = encoder.getDistance();
    angleDifference = turretAngle - targetAngle;
    pidSpeed = rController.calculate(angleDifference);

    turret.setSpeed(pidSpeed);
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

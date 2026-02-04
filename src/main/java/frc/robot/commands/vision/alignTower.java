// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.Meters;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
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

  PIDController rController = new PIDController(0.1, 0, 0);
  PIDController xController = new PIDController(0.1, 0, 0);
  PIDController yController = new PIDController(0.1, 0, 0);

  Translation2d targetPose;
  Translation2d poseDifference;
  Double targetXPoseDifference;
  Double targetYPoseDifference;

  Rotation2d targetAngle;
  Rotation2d currentAngle;
  Rotation2d angleDifference;

  double rSpeed;
  double xSpeed;
  double ySpeed;

  /** Creates a new alignTower. */
  public alignTower(SS_Vision ss_vision, SS_Drivetrain ss_drivetrain) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_vision);
    this.vision = ss_vision;
    this.drivetrain = ss_drivetrain;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    targetPose = drivetrain.towerPose;

    estimatedPose = drivetrain.poseEstimator.getEstimatedPosition().getTranslation();
    poseDifference = targetPose.minus(estimatedPose);
    targetAngle = poseDifference.getAngle();

    currentAngle = drivetrain.pidgey.getRotation2d();
    angleDifference = currentAngle.minus(targetAngle);
    targetXPoseDifference = poseDifference.getMeasureY().in(Meters);
    targetYPoseDifference = poseDifference.getMeasureX().in(Meters);

    rSpeed = rController.calculate(angleDifference.getDegrees());
    xSpeed = xController.calculate(targetXPoseDifference);
    ySpeed = yController.calculate(targetYPoseDifference);

    drivetrain.applyRequest(() -> 
        drive.withVelocityX(xSpeed)
        .withVelocityY(ySpeed)
        .withRotationalRate(rSpeed));
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

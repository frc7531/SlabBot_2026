// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.MetersPerSecond;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SS_Drivetrain;
import frc.robot.subsystems.SS_Vision;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class aimTowardsTarget extends Command {
  /** Creates a new findTarget. */
  public SS_Vision vision;
  public SS_Drivetrain drivetrain;

  PIDController rController = new PIDController(0.1, 0, 0);
  Translation2d estimatedPose;
  Translation2d targetPose;
  Translation2d hubPose = new Translation2d(4.625594, 4.034536);
  Rotation2d targetAngle;
  double speed;
  public SwerveRequest.RobotCentric driverequest = new SwerveRequest.RobotCentric();
  public CommandXboxController controller;

  public aimTowardsTarget(SS_Vision ss_vision, SS_Drivetrain ss_drivetrain, CommandXboxController joystick) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(ss_vision, ss_drivetrain);
    this.vision = ss_vision;
    this.controller = joystick;
    this.drivetrain = ss_drivetrain;
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
    estimatedPose = drivetrain.poseEstimator.getEstimatedPosition().getTranslation();
    targetPose = hubPose.minus(estimatedPose);
    targetAngle = targetPose.getAngle();
    speed = -rController.calculate(targetAngle.getDegrees() - drivetrain.pidgey.getYaw().getValueAsDouble());
    driverequest.withVelocityX(-controller.getLeftY()*TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                .withVelocityY(-controller.getLeftX()*TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                .withRotationalRate(speed);
    drivetrain.setControl(driverequest);
    System.out.println("estimatedPose: " + estimatedPose.toString());
    System.out.println("diffPose: " + targetPose.toString());
    System.out.println("targetAngle: " + targetAngle);
    System.out.println("currentAngle: " + drivetrain.pidgey.getYaw().getValueAsDouble());
    System.out.println("speed: " + speed);
    System.out.println();
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

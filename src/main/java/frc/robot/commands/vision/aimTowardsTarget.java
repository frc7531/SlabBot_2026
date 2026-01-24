// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.vision;

import static edu.wpi.first.units.Units.MetersPerSecond;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.RotationsPerSecond;

import org.ejml.interfaces.linsol.ReducedRowEchelonForm;

import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  PIDController rController = new PIDController(0.05, 0, 0);

  Translation2d estimatedPose;
  Translation2d targetPose;

  Translation2d blueHubPose = new Translation2d(4.625594, 4.034663); //11.915394 4.03479
  Translation2d redHubPose = new Translation2d(11.915394, 4.034663);

  Translation2d blueRightPose = new Translation2d(1.5, 1.5);
  Translation2d blueLeftPose = new Translation2d(1.5, 6.57);
  Translation2d redRightPose = new Translation2d(15.04, 6.57);
  Translation2d redLeftPose = new Translation2d(15.04, 1.5);

  Translation2d focusPose;
  public Rotation2d targetAngle;
  double currentAngle;

    Translation2d hubTargetPose;
    public Rotation2d hubTargetAngle;

    double phaseShift;

  double pidSpeed;
  double angleDifference;
  
  Field2d hubField = new Field2d();

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

        switch (drivetrain.alliance) {
            case Red:
                focusPose = redHubPose;
                phaseShift = -180;
                break;
            case Blue:
                focusPose = blueHubPose;
                phaseShift = 0;
                break;
        }

        hubTargetPose = focusPose.minus(estimatedPose);
        hubTargetAngle = hubTargetPose.getAngle();

        switch (drivetrain.alliance) {
            case Red:
                switch ((int) Math.signum(Math.abs(hubTargetAngle.getDegrees()) - 90)) {
                    case 1: // Red side of barrier
                        focusPose = redHubPose;
                        break;
                    case -1: // Neutral side of barrier
                        switch ((int) Math.signum(hubTargetAngle.getDegrees())) {
                            case 1: // Blue left side
                                focusPose = redLeftPose;
                                break;
                            case -1: // Blue right side
                                focusPose = redRightPose;
                                break;
                        }
                        break;
                }
                break;
            case Blue:
                switch ((int) Math.signum(90 - Math.abs(hubTargetAngle.getDegrees()))) {
                    case 1: // Blue side of barrier
                        focusPose = blueHubPose;
                        break;
                    case -1: // Neutral side of barrier
                        switch ((int) Math.signum(hubTargetAngle.getDegrees())) {
                            case 1: // Blue right side
                                focusPose = blueRightPose;
                                break;
                            case -1: // Blue left side
                                focusPose = blueLeftPose;
                                break;
                        }
                        break;
                }
                break;
        }  

        targetPose = focusPose.minus(estimatedPose);
        targetAngle = targetPose.getAngle();//.plus(new Rotation2d(phaseShift));
        currentAngle = drivetrain.pidgey.getYaw().getValueAsDouble();// + phaseShift;

        angleDifference = targetAngle.getDegrees() - currentAngle;

        //targetAngle.getDegrees() - drivetrain.pidgey.getYaw().getValueAsDouble();
        pidSpeed = -rController.calculate(angleDifference);
        driverequest.withVelocityX(-controller.getLeftY()*TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                    .withVelocityY(-controller.getLeftX()*TunerConstants.kSpeedAt12Volts.in(MetersPerSecond))
                    //.withRotationalRate(pidSpeed);
                    .withRotationalRate(-controller.getRightX()*RotationsPerSecond.of(0.75).in(RadiansPerSecond));
        drivetrain.setControl(driverequest);
        System.out.println("estimatedPose: " + estimatedPose.toString());
        System.out.println("diffPose: " + targetPose.toString());
        System.out.println("targetAngle: " + targetAngle);
        System.out.println("currentAngle: " + drivetrain.pidgey.getYaw().getValueAsDouble());
        System.out.println("speed: " + pidSpeed);
        System.out.println();

        hubField.setRobotPose(new Pose2d(focusPose, new Rotation2d(0)));
        SmartDashboard.putData("hubField", hubField);
        SmartDashboard.putNumber("targetAngle", targetAngle.getDegrees());
        SmartDashboard.putNumber("currentAngle", drivetrain.pidgey.getYaw().getValueAsDouble());
        SmartDashboard.putNumber("angleDifference", angleDifference);
        SmartDashboard.putNumber("speed", pidSpeed);
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

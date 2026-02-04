// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.moveActuator;
import frc.robot.commands.vision.aimTurretToTarget;
import frc.robot.commands.vision.manualShooter;
import frc.robot.commands.vision.manualTurret;
import frc.robot.commands.vision.startThroat;
import frc.robot.commands.vision.stopShooter;
import frc.robot.commands.vision.stopThroat;
import frc.robot.commands.vision.stopTurret;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.SS_Drivetrain;
import frc.robot.subsystems.SS_Shooter;
import frc.robot.subsystems.SS_Throat;
import frc.robot.subsystems.SS_Turret;
import frc.robot.subsystems.SS_Vision;

public class RobotContainer {
    private double MaxSpeed = 1.0 * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    public final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    public final SS_Vision vision = new SS_Vision();
    public final SS_Drivetrain drivetrain = TunerConstants.createDrivetrain();
    public final SS_Shooter shooter = new SS_Shooter();
    public final SS_Turret turret = new SS_Turret();
    public final SS_Throat throat = new SS_Throat();
    public aimTurretToTarget aimCommand = new aimTurretToTarget(drivetrain, turret);
    public Command drivetrainDefault = drivetrain.applyRequest(() ->
        drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
             .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
             .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
    );
    public manualTurret turretForward = new manualTurret(turret).withSpeed(0.1);
    public manualTurret turretReverse = new manualTurret(turret).withSpeed(-0.1);
    public stopTurret stopCommand = new stopTurret(turret);
    public manualShooter shootCommand = new manualShooter(shooter);
    public moveActuator moveActuator = new moveActuator(shooter);
    public stopShooter stopSCommand = new stopShooter(shooter);
    //public stopThroat stopTCommand = new stopThroat(throat);
    //public startThroat startCommand = new startThroat(throat);


    public RobotContainer() {
        CommandScheduler.getInstance().onCommandInitialize(command -> System.out.println("[CMD INIT] " + command.getName()));
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrainDefault
        );

        turret.setDefaultCommand(aimCommand);
        shooter.setDefaultCommand(stopSCommand);
        //throat.setDefaultCommand(stopTCommand);

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        // final var idle = new SwerveRequest.Idle();
        // RobotModeTriggers.disabled().whileTrue(
        //     drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        // );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));
        joystick.y().whileTrue(drivetrain.pigeonCommand());
        joystick.x().whileTrue(aimCommand);

        // Reset the field-centric heading on left bumper press.
        joystick.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick2.y().whileTrue(turretForward);
        joystick2.x().whileTrue(turretReverse);
        joystick2.a().whileTrue(shootCommand);//.alongWith(startCommand));
        joystick2.b().whileTrue(moveActuator);
    }

    public Command getAutonomousCommand() {
        // Simple drive forward auton
        final var idle = new SwerveRequest.Idle();
        return Commands.sequence(
            // Reset our field centric heading to match the robot
            // facing away from our alliance station wall (0 deg).
            drivetrain.runOnce(() -> drivetrain.seedFieldCentric(Rotation2d.kZero)),
            // Then slowly drive forward (away from us) for 5 seconds.
            drivetrain.applyRequest(() ->
                drive.withVelocityX(0.5)
                    .withVelocityY(0)
                    .withRotationalRate(0)
            )
            .withTimeout(5.0),
            // Finally idle for the rest of auton
            drivetrain.applyRequest(() -> idle)
        );
    }
}

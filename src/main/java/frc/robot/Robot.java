// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.HootAutoReplay;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;

    /* log and replay timestamp and joystick data */
    private final HootAutoReplay m_timeAndJoystickReplay = new HootAutoReplay()
        .withTimestampReplay()
        .withJoystickReplay();

 
    public Robot() {
        m_robotContainer = new RobotContainer();
        DataLogManager.logNetworkTables(true);
        DataLogManager.start();
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run(); 
        m_timeAndJoystickReplay.update();
        // m_robotContainer.drivetrain.updateOdometry();
        NetworkTableInstance.getDefault().getTable("limelight-antigua").getEntry("<throttle_set>").setNumber(5);
        NetworkTableInstance.getDefault().getTable("limelight-barbuda").getEntry("<throttle_set>").setNumber(5);
        NetworkTableInstance.getDefault().getTable("limelight-antigua").getEntry("pipeline").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight-barbuda").getEntry("pipeline").setNumber(0);
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {
        NetworkTableInstance.getDefault().getTable("limelight-barbuda").getEntry("<throttle_set>").setNumber(9999);
        NetworkTableInstance.getDefault().getTable("limelight-antigua").getEntry("<throttle_set>").setNumber(9999);
    }

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().schedule(m_autonomousCommand);
        }
    }

    @Override
    public void autonomousPeriodic() {
        NetworkTableInstance.getDefault().getTable("limelight-barbuda").getEntry("<throttle_set>").setNumber(5);
        NetworkTableInstance.getDefault().getTable("limelight-antigua").getEntry("<throttle_set>").setNumber(5);
        NetworkTableInstance.getDefault().getTable("limelight-antigua").getEntry("pipeline").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight-barbuda").getEntry("pipeline").setNumber(0);
    }

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }
    }

    @Override
    public void teleopPeriodic() {
        NetworkTableInstance.getDefault().getTable("limelight-barbuda").getEntry("<throttle_set>").setNumber(5);
        NetworkTableInstance.getDefault().getTable("limelight-antigua").getEntry("<throttle_set>").setNumber(5);
        NetworkTableInstance.getDefault().getTable("limelight-antigua").getEntry("pipeline").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight-barbuda").getEntry("pipeline").setNumber(0);
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {
        NetworkTableInstance.getDefault().getTable("limelight-barbuda").getEntry("<throttle_set>").setNumber(5);
        NetworkTableInstance.getDefault().getTable("limelight-antigua").getEntry("<throttle_set>").setNumber(5);
        NetworkTableInstance.getDefault().getTable("limelight-antigua").getEntry("pipeline").setNumber(0);
        NetworkTableInstance.getDefault().getTable("limelight-barbuda").getEntry("pipeline").setNumber(0);
    }

    @Override
    public void testExit() {}

    @Override
    public void simulationPeriodic() {}
}

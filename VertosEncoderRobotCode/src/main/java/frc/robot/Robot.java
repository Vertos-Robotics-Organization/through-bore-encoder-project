// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.VendorFiles.main.java.com.vertos.encoder.ParentCANSense;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

 private ParentCANSense encoder = new ParentCANSense(0, true);

  private final RobotContainer m_robotContainer;

  public Robot() {
    m_robotContainer = new RobotContainer();
    encoder.start();
  }

  @Override
  public void robotPeriodic() {
    
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Encoder Absolute Rotations", encoder.getAbsRotations());
    SmartDashboard.putNumber("Encoder Velocity", encoder.getSensorVelocityRPS());
    SmartDashboard.putNumber("Encoder Accel", encoder.getSensorAccelerationRPS2());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}

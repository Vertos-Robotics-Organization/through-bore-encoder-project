// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.hardware.CANcoder;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.VendorFiles.main.java.com.vertos.encoder.CANSense;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

 private CANSense encoder = new CANSense(0, true);
 private CANcoder canCoder = new CANcoder(2);

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
    SmartDashboard.putBoolean("Encoder Fault: Hardware Failure", encoder.getFault_Hardware());
    SmartDashboard.putBoolean("Encoder Fault: Boot During Enable", encoder.getFault_BootDuringEnable());
    SmartDashboard.putBoolean("Encoder Fault: Loop Overrun", encoder.getFault_LoopOverrun());
    SmartDashboard.putBoolean("Encoder Fault: Bad Magnet", encoder.getFault_BadMagnet());
    SmartDashboard.putBoolean("Encoder Fault: General CAN Fault", encoder.getFault_CANGeneral());
    SmartDashboard.putBoolean("Encoder Fault: Momentary CAN Bus Loss", encoder.getFault_MomentaryCanBusLoss());
    SmartDashboard.putBoolean("Encoder Fault: CAN Clogged", encoder.getFault_CANClogged());
    SmartDashboard.putBoolean("Encoder Fault: Rotation Overspeed", encoder.getFault_RotationOverspeed());
    SmartDashboard.putBoolean("Encoder Fault: Under Volted", encoder.getFault_UnderVolted());
    SmartDashboard.putBoolean("Encoder Connected", encoder.isConnected());
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

    encoder.setPosition(2.0);
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

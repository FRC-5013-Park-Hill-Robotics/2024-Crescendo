// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Optional;

import edu.wpi.first.util.datalog.BooleanLogEntry;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.AutoAdjustAngle;
import frc.robot.subsystems.Limelight;
import com.ctre.phoenix6.Orchestra;
import com.ctre.phoenix6.hardware.TalonFX;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private RobotContainer m_robotContainer;
  BooleanLogEntry myBooleanLog;
  DoubleLogEntry myDoubleLog;
  StringLogEntry myStringLog;


  @Override
  public void robotInit() {
    m_robotContainer = new RobotContainer();
    //DataLogManager.start();

  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("Match Time", DriverStation.getMatchTime());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    checkUpdateAlliance();
    final TalonFX frontleftS = new TalonFX(2);
    Orchestra m_Orchestra = new Orchestra();
      
      // Add motors to orchestra
      m_Orchestra.addInstrument(frontleftS);

      // Load chrp file
      var status = m_Orchestra.loadMusic("9NoteAuto.chrp");

      if(!status.isOK()) {
        m_Orchestra.play();
    }

    m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_robotContainer.getLauncherShoulder().holdCommand().schedule();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}
    

  @Override
  public void autonomousExit() {
  }

  @Override
  public void teleopInit() {
     checkUpdateAlliance();
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    m_robotContainer.getLauncherShoulder().holdCommand().schedule();
    //m_robotContainer.getLauncherShoulder().setDefaultCommand(new AutoAdjustAngle(m_robotContainer.getLauncherRollers(), m_robotContainer.getLauncherShoulder()));

  }

  @Override
  public void teleopPeriodic() {
   
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {
  }

  @Override
  public void testExit() {
  }

  @Override
  public void simulationPeriodic() {
  }

  private void checkUpdateAlliance() {
    
    Optional<Alliance> alliance = DriverStation.getAlliance();
    if (DriverStation.isDSAttached() && alliance.isPresent()) {
      Limelight frontLL = m_robotContainer.getFrontLimelight();
     // Limelight backLL = m_robotContainer.getBackLimelight();
      frontLL.setAlliance(alliance.get());
      //backLL.setAlliance(alliance.get());
    }

    //zero gyroscope might break orientation?
    
  }
}

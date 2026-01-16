// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj.DataLogManager;


public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private ShuffleboardTab tab;

  private GenericEntry matchTimeEntry;
  private GenericEntry voltsEntry;
  private GenericEntry ampsEntry;

  private boolean ppConfigured = false;

  private Command initAuto;

  public Robot() {
    /*Start Logging */
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
 
    /*Create Robot Contrainer */
    m_robotContainer = new RobotContainer();
    
    /*Initialize base Shuffleboard tab */
    tab = Shuffleboard.getTab("tab");
    matchTimeEntry = tab.add("Match time",0.0).getEntry();
    voltsEntry = tab.add("Volts",0.0).getEntry();
    ampsEntry = tab.add("Amps",0.0).getEntry();

    /* Create a default auto to initialize Path Planner  */
    initAuto = new PathPlannerAuto("initAuto").ignoringDisable(true);

  }

  @Override
  public void robotPeriodic() {
    /* Run the command scheduler */
    CommandScheduler.getInstance().run();

    /* Update base Shuffleboard tab with Roborio data  */
    matchTimeEntry.setDouble(DriverStation.getMatchTime());
    voltsEntry.setDouble(RoboRioDataJNI.getVInVoltage());
    ampsEntry.setDouble(RoboRioDataJNI.getVInCurrent());
  }

  @Override
  public void disabledInit() {
    /* Fire off the Pathplanner Auto to initialize Path Planner (Work around a bug) */
    if (!ppConfigured) {
      initAuto.schedule();
      ppConfigured = true;
    }
    
    SmartDashboard.putBoolean("Teleop", false);
    SmartDashboard.putBoolean("Auto", false);

  }

  @Override
  public void disabledPeriodic() {
    m_robotContainer.updateAutonCommand();
  }

  @Override
  public void disabledExit() {
    /*Clear our the fake Path Planner Auto Initialize  */
    initAuto.cancel();
  }

  @Override
  public void autonomousInit() {
    SmartDashboard.putBoolean("Auto", true);

    m_autonomousCommand = Commands.waitSeconds(0.01).andThen(m_robotContainer.getAutonomousCommand());

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
    SmartDashboard.putBoolean("Teleop", true);

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

  @Override
  public void simulationPeriodic() {
    
  }
}

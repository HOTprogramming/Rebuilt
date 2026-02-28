// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import edu.wpi.first.hal.simulation.RoboRioDataJNI;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.util.HubTracker;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import static edu.wpi.first.units.Units.Seconds;


public class Robot extends TimedRobot {

  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private ShuffleboardTab tab;

  private GenericEntry matchTimeEntry;

  private DoublePublisher timeRemainingInCurrentShift;
  private BooleanPublisher hubActive;
  private BooleanPublisher hubActiveNext;

  private boolean ppConfigured = false;

  private static final double SIM_DISABLED_SECONDS = 10;
  private static final double SIM_AUTO_SECONDS = 10;
  private static final double SIM_TELEOP_SECONDS = 10;

  private final boolean ciForceModes = "true".equalsIgnoreCase(System.getenv("CI_FORCE_MODES"))
      && "true".equalsIgnoreCase(System.getenv("GITHUB_ACTIONS"));
  private double simStartTime = -1.0;
  private SimMode currentSimMode = null;
    
  private enum SimMode {
    DISABLED,
    AUTO,
    TELEOP
  }

  public Robot() {


    /*Start Logging */
    DataLogManager.start();
    DriverStation.startDataLog(DataLogManager.getLog(), true);
	DriverStation.silenceJoystickConnectionWarning(true);
  
    /*Create Robot Contrainer */
    m_robotContainer = new RobotContainer();
    
    /*Initialize base Shuffleboard tab */
    tab = Shuffleboard.getTab("tab");
    matchTimeEntry = tab.add("Match time",0.0).getEntry();

    NetworkTableInstance inst = NetworkTableInstance.getDefault();
    NetworkTable table = inst.getTable("HOT/Drive");

    timeRemainingInCurrentShift = table.getDoubleTopic("Hub Time Remaining").publish();
	hubActive = table.getBooleanTopic("Hub Active").publish();
	hubActiveNext = table.getBooleanTopic("Hub Active Next").publish();
  }

    private void UpdateTelemetry() {
    try{
      timeRemainingInCurrentShift.set(HubTracker.timeRemainingInCurrentShift().get().abs(Seconds));
	  hubActive.set(HubTracker.isActive());
      hubActiveNext.set(HubTracker.isActiveNext());
    } catch(Exception e) {
    }
  }
    

  @Override
  public void robotPeriodic() {
    /* Run the command scheduler */
    CommandScheduler.getInstance().run();

    /* Update base Shuffleboard tab with Roborio data  */
    matchTimeEntry.setDouble(DriverStation.getMatchTime());
	double batteryVoltage = RobotController.getBatteryVoltage();
	SmartDashboard.putNumber("Battery Voltage", batteryVoltage);


    UpdateTelemetry(); 
  }

  @Override
  public void disabledInit() {
    /* Fire off the Pathplanner Auto to initialize Path Planner (Work around a bug) */
    if (!ppConfigured) {
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
  }

  @Override
  public void autonomousInit() {
    SmartDashboard.putBoolean("Auto", true);

    m_autonomousCommand = Commands.waitSeconds(0.01).andThen(m_robotContainer.getAutonomousCommand());

    if (m_autonomousCommand != null) {
      CommandScheduler.getInstance().schedule(m_autonomousCommand);
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
    if (!ciForceModes) {
      return;
    }

    if (simStartTime < 0) {
      simStartTime = Timer.getFPGATimestamp();
    }

    double elapsed = Timer.getFPGATimestamp() - simStartTime;

    if (elapsed < SIM_DISABLED_SECONDS) {
      setDriverStationState(SimMode.DISABLED);
    } else if (elapsed < SIM_DISABLED_SECONDS + SIM_AUTO_SECONDS) {
      setDriverStationState(SimMode.AUTO);
    } else if (elapsed < SIM_DISABLED_SECONDS + SIM_AUTO_SECONDS + SIM_TELEOP_SECONDS) {
      setDriverStationState(SimMode.TELEOP);
    } else {
      setDriverStationState(SimMode.TELEOP);
    }
  }

  @Override
  public void simulationInit() {
    if (ciForceModes) {
      simStartTime = Timer.getFPGATimestamp();
      setDriverStationState(SimMode.DISABLED);
    }
  }

  private void setDriverStationState(SimMode mode) {
    if (mode != currentSimMode) {
      System.out.println("CI sim mode -> " + mode);
      currentSimMode = mode;
    }

    boolean enabled = mode != SimMode.DISABLED;
    boolean autonomous = mode == SimMode.AUTO;

    DriverStationSim.setEnabled(enabled);
    DriverStationSim.setAutonomous(autonomous);
    DriverStationSim.setTest(false);
    DriverStationSim.notifyNewData();
  }
}

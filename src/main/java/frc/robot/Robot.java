package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
    private Command m_autonomousCommand;

    private final RobotContainer m_robotContainer;
	public final RobotState state = new RobotState();

    public Robot() {
		DataLogManager.start();
		DriverStation.startDataLog(DataLogManager.getLog(), true);
		DriverStation.silenceJoystickConnectionWarning(true);

        m_robotContainer = new RobotContainer();
    }

    @Override
    public void robotPeriodic() {
		RobotState.climberCurrent = m_robotContainer.climber.m_ClimbKraken.getTotalCurrent();
		RobotState.intakeCurrent = m_robotContainer.intake.mIntakeKraken.getTotalCurrent();
		RobotState.indexerCurrent = m_robotContainer.indexer.m_IndexKraken.getTotalCurrent();
		RobotState.shooterCurrent = m_robotContainer.shooter.mShooterKraken.getTotalCurrent();
		RobotState.update();
        CommandScheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
		CommandScheduler.getInstance().cancelAll();

		SmartDashboard.putBoolean("Auton", false);
		SmartDashboard.putBoolean("Teleop", false);
	}

    @Override
    public void disabledPeriodic() {}

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
		SmartDashboard.putBoolean("Auton", true);
	}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {
        if (m_autonomousCommand != null) {
            CommandScheduler.getInstance().cancel(m_autonomousCommand);
        }

		SmartDashboard.putBoolean("Teleop", true);
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
    public void simulationPeriodic() {}
}
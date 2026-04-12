package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Index extends SubsystemBase {
	public final IndexKraken m_IndexKraken = new IndexKraken();

	public Index() {

	}

	@Override
	public void periodic() {
		m_IndexKraken.updateStats();
	}

	public Command indexOnCommand() {
		return runOnce(() -> 
		{m_IndexKraken.startKicker();
		m_IndexKraken.startFloor();});
	}

	public Command reverseIndexCommand() {
		return runOnce(() -> m_IndexKraken.reverseIndex());
	}

	public Command indexOffCommand() {
		return runOnce(() -> 
		{m_IndexKraken.stopKicker();
		m_IndexKraken.stopFloor();});
	}

	public Command turnOnKicker() {
		return runOnce(() -> m_IndexKraken.startKicker());
	}	
	
	public Command turnOnFloor() {
		return runOnce(() -> m_IndexKraken.startFloor());
	}

	public Command index() {
		return Commands.sequence();
	}

	/**
	 * Returns the average kicker wheel speed (rps). Used by the shooter's
	 * kicker-speed-based hood offset to detect when balls are loading slowly.
	 */
	public double getAverageKickerSpeed() {
		return m_IndexKraken.getAverageKickerSpeed();
	}
}
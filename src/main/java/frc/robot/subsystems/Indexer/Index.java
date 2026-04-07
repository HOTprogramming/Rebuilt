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
		return Commands.sequence(
			runOnce(() -> m_IndexKraken.startKicker()),
			// Commands.waitSeconds(0.10),
			runOnce(() -> m_IndexKraken.startFloor()));
	}

	public Command indexOffCommand() {
		return Commands.sequence(
			runOnce(() -> m_IndexKraken.stopFloor()),
			// Commands.waitSeconds(0.10),
			runOnce(() -> m_IndexKraken.stopKicker()));
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
}
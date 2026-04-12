package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Shooter.Shooter;

public class Index extends SubsystemBase {
	public final IndexKraken m_IndexKraken = new IndexKraken();

	public Index() {

	}

	@Override
	public void periodic() {
		m_IndexKraken.updateStats();
	}

	/**
	 * Starts the indexer and the shooter's shot timer when the floor begins
	 * feeding. Sequence: kicker on → 100ms spin-up → floor on + shot timer.
	 * The shot timer ensures the hood offset system begins tracking from the
	 * instant balls start feeding into the kicker.
	 */
	public Command indexOnCommand(Shooter shooter) {
		return Commands.sequence(
			runOnce(() -> m_IndexKraken.startKicker()),
			Commands.waitSeconds(0.1),
			runOnce(() -> {
				m_IndexKraken.startFloor();
				shooter.startShotTimerNow();
			}));
	}

	public Command indexOffCommand() {
		return Commands.sequence(
			runOnce(() -> m_IndexKraken.stopFloor()),
			Commands.waitSeconds(0.1),
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

	/**
	 * Returns the average kicker wheel speed (rps). Used by the shooter's
	 * kicker-speed-based hood offset to detect when balls are loading slowly.
	 */
	public double getAverageKickerSpeed() {
		return m_IndexKraken.getAverageKickerSpeed();
	}
}
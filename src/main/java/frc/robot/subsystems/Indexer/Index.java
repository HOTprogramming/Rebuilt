package frc.robot.subsystems.Indexer;

import edu.wpi.first.wpilibj2.command.Command;
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
		return runOnce(() -> m_IndexKraken.startIndex());
	}

	public Command indexOffCommand() {
		return runOnce(() -> m_IndexKraken.stop());
	}
}
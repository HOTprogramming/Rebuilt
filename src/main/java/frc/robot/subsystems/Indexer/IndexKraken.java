package frc.robot.subsystems.indexer;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.util.Kraken;

public class IndexKraken {
	public Kraken m_leftKickKraken = new Kraken(IndexConstants.kLeftKickID, IndexConstants.IndexConfigs
		.indexWheelConfigs(), InvertedValue.Clockwise_Positive, "Indexer", "LeftKick");
	public Kraken m_rightKickKraken = new Kraken(IndexConstants.kRightKickID, IndexConstants.IndexConfigs
		.indexWheelConfigs(), InvertedValue.Clockwise_Positive, "Indexer", "RightKick");
	public Kraken m_floorKraken = new Kraken(IndexConstants.kFloorID, IndexConstants.IndexConfigs
		.indexWheelConfigs(), InvertedValue.CounterClockwise_Positive, "Indexer", "Floor");

	private static IndexKraken instance = null;

	public IndexKraken() {
		m_floorKraken.setup();
		m_leftKickKraken.setup();
		m_rightKickKraken.setup();
	}

	public static IndexKraken getInstance() {
		return instance = instance == null ? new IndexKraken() : instance;
	}

	public void updateStats() {
		m_floorKraken.update();
		m_leftKickKraken.update();
		m_rightKickKraken.update();
	}

	public void startIndex() {
		m_rightKickKraken.setVelocity(IndexConstants.kickerSpeed, Kraken.VelocityControlType.VelocityTorqueCurrentFOC);
		m_leftKickKraken.setVelocity(IndexConstants.kickerSpeed, Kraken.VelocityControlType.VelocityTorqueCurrentFOC);
		m_floorKraken.setVelocity(IndexConstants.floorSpeed, Kraken.VelocityControlType.VelocityTorqueCurrentFOC, 1);
	
	}

	public void stop() {
		m_floorKraken.stop();
		m_leftKickKraken.stop();
		m_rightKickKraken.stop(); 
 
	}
}
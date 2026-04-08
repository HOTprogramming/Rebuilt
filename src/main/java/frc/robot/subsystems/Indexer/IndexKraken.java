package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.signals.InvertedValue;
import frc.util.Kraken;

public class IndexKraken {
	public Kraken m_leftKickKraken = new Kraken(IndexConstants.kLeftKickID, IndexConstants.IndexConfigs
		.kickerConfigs(), InvertedValue.Clockwise_Positive, "Indexer", "LeftKick");
	public Kraken m_rightKickKraken = new Kraken(IndexConstants.kRightKickID, IndexConstants.IndexConfigs
		.kickerConfigs(), InvertedValue.Clockwise_Positive, "Indexer", "RightKick");
	public Kraken m_floorKraken = new Kraken(IndexConstants.kFloorID, IndexConstants.IndexConfigs
		.floorConfigs(), InvertedValue.CounterClockwise_Positive, "Indexer", "Floor");

	public IndexKraken() {
		m_floorKraken.setup();
		m_leftKickKraken.setup();
		m_rightKickKraken.setup();
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

	public void startFloor() {
		m_floorKraken.setVelocity(IndexConstants.floorSpeed, Kraken.VelocityControlType.VelocityTorqueCurrentFOC, 1);
	}	
	
	public void startKicker() {
		m_rightKickKraken.setVelocity(IndexConstants.kickerSpeed, Kraken.VelocityControlType.VelocityTorqueCurrentFOC);
		m_leftKickKraken.setVelocity(IndexConstants.kickerSpeed, Kraken.VelocityControlType.VelocityTorqueCurrentFOC);
	}

	public void stopFloor() {
		m_floorKraken.stop();
	}	
	
	public void stopKicker() {
		m_rightKickKraken.stop();
		m_leftKickKraken.stop();
	}

	public void stop() {
		m_floorKraken.stop();
		m_leftKickKraken.stop();
		m_rightKickKraken.stop(); 
 
	}
		public double getTotalCurrent(){
		return m_floorKraken.getSupplyCurrent() + m_leftKickKraken.getSupplyCurrent() + m_rightKickKraken.getSupplyCurrent();
	}
}
package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeConstants.IntakeConfigs;
import frc.util.Kraken;

public class IndexKraken {
	public Kraken m_leftKickKraken = new Kraken(IndexConstants.kLeftKickID, IndexConstants.IndexConfigs
		.indexWheelConfigs(), InvertedValue.Clockwise_Positive, "Indexer", "LeftKick");
	public Kraken m_rightKickKraken = new Kraken(IndexConstants.kRightKickID, IndexConstants.IndexConfigs
		.indexWheelConfigs(), InvertedValue.Clockwise_Positive, "Indexer", "RightKick");
	public Kraken m_floorKraken = new Kraken(IndexConstants.kFloorID, IndexConstants.IndexConfigs
		.indexWheelConfigs(), InvertedValue.Clockwise_Positive, "Indexer", "Floor");

	private static IndexKraken instance = null;

	public IndexKraken() {
		m_floorKraken.setup();
		m_leftKickKraken.setup();
		m_rightKickKraken.setup();

		m_rightKickKraken.setFollower(IndexConstants.kLeftKickID, MotorAlignmentValue.Aligned);
	}

	public static IndexKraken getInstance() {
		return instance = instance == null ? new IndexKraken() : instance;
	}

	public void updateStats() {
		m_floorKraken.refresh();
		m_leftKickKraken.refresh();
		m_rightKickKraken.refresh();
	}

	public void startIndex() {
		m_floorKraken.setVelocity(IndexConstants.floorSpeed, Kraken.VelocityControlType.VelocityVoltage, 1);
		m_leftKickKraken.setVelocity(IndexConstants.kickerSpeed, Kraken.VelocityControlType.VelocityVoltage);		
	}

	public void stop() {
		m_floorKraken.stop();
		m_leftKickKraken.stop(); 
	}
}
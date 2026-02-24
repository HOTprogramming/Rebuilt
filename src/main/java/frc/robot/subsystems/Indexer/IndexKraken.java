package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.util.Kraken;

public class IndexKraken {
	public Kraken m_leftKickKraken = new Kraken(IndexConstants.kLeftKickID, IndexConstants.IndexConfigs
		.indexWheelConfigs(), InvertedValue.Clockwise_Positive, "Indexer", "LeftKick");
	public Kraken m_rightKickKraken = new Kraken(IndexConstants.kRightKickID, IndexConstants.IndexConfigs
		.indexWheelConfigs(), InvertedValue.Clockwise_Positive, "Indexer", "RightKick");
	public Kraken m_floorKraken = new Kraken(IndexConstants.kFloorID, IndexConstants.IndexConfigs
		.indexWheelConfigs(), InvertedValue.Clockwise_Positive, "Indexer", "Floor");
	public Kraken m_loaderKraken = new Kraken(IndexConstants.kLoaderID, IndexConstants.IndexConfigs
		.indexWheelConfigs(), InvertedValue.Clockwise_Positive, "Indexer", "Loader");

	private static IndexKraken instance = null;

	public IndexKraken() {
		m_floorKraken.setup();
		m_loaderKraken.setup();
		m_leftKickKraken.setup();
		m_rightKickKraken.setup();
	}

	public static IndexKraken getInstance() {
		return instance = instance == null ? new IndexKraken() : instance;
	}

	public void updateStats() {
		m_floorKraken.refresh();
		m_loaderKraken.refresh();
		m_leftKickKraken.refresh();
		m_rightKickKraken.refresh();
	}

	public void startIndex(double speed) {
		// m_floorKraken.setVelocity(speed, Kraken.VelocityControlType.VelocityVoltage);
		// m_loaderKraken.setVelocity(speed, Kraken.VelocityControlType.VelocityVoltage);
		// m_leftKickKraken.setVelocity(speed, Kraken.VelocityControlType.VelocityVoltage);
		// m_rightKickKraken.setVelocity(speed, Kraken.VelocityControlType.VelocityVoltage);
		if(speed > 0){
			SmartDashboard.putBoolean("Pulseing?", true);
			m_floorKraken.motor.setControl(new DutyCycleOut(.8));
			m_loaderKraken.motor.setControl(new DutyCycleOut(.8));
			m_leftKickKraken.motor.setControl(new DutyCycleOut(.93));
			m_rightKickKraken.motor.setControl(new DutyCycleOut(.93));
		} else {
			SmartDashboard.putBoolean("Pulseing?", false);
			m_floorKraken.motor.setControl(new DutyCycleOut(1));
			m_loaderKraken.motor.setControl(new DutyCycleOut(1));
			m_leftKickKraken.motor.setControl(new DutyCycleOut(.93));
			m_rightKickKraken.motor.setControl(new DutyCycleOut(.93));
		}
		
	}

	public void stop() {
		m_leftKickKraken.stop();
		m_rightKickKraken.stop();
		m_floorKraken.stop();
		m_loaderKraken.stop();
	}
}
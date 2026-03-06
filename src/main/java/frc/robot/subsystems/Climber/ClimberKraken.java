package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.util.Kraken;
import frc.util.KrakenWithCancoder;

public class ClimberKraken {
	private static ClimberKraken instance = null;
	public DigitalInput beambreak;


	public Kraken m_climberKraken = new KrakenWithCancoder(ClimberConstants.kClimbermotorID, ClimberConstants.kClimbEncoder, ClimberConstants.ClimberConfigs
		.elbowConfigs(), ClimberConstants.encoderConfigs, "Climber", "ClimberMotor");
	public Kraken m_spinnerKraken = new Kraken(ClimberConstants.kClimberspinnerID, ClimberConstants.ClimberConfigs
		.wristConfigs(), "Climber", "SpinnerMotor");

	public ClimberKraken() {
		m_climberKraken.setup();
		m_spinnerKraken.setup();
		beambreak = new DigitalInput(0);
	}

	public static ClimberKraken getInstance() {
		return instance = instance == null ? new ClimberKraken() : instance;
	}

	public void updateStats() {
		m_climberKraken.refresh();
		m_spinnerKraken.refresh();
	}

	public boolean getBeambreakStatus() {
		return beambreak.get();
	}
	public double getSpinPosition() {
		return m_spinnerKraken.position.getValueAsDouble();
	}
	public double getClimbPosition() {
		return m_climberKraken.position.getValueAsDouble();
	}

	public void runManual(double percentOutputElbow, double percentOutputWrist) {
		m_climberKraken.motor.setControl(new DutyCycleOut(percentOutputElbow));
		m_spinnerKraken.motor.setControl(new DutyCycleOut(percentOutputWrist));
	}

	public void runPosition(double desiredRotations) {
		m_climberKraken.setPosition(desiredRotations, Kraken.PositionControlType.PositionVoltage, 0);
	}

	public void spinrunPosition(double desiredRotations) {
		m_spinnerKraken.setPosition(desiredRotations, Kraken.PositionControlType.PositionVoltage, 0);
	}

	public void spinRunVelocity(double desiredVelocity) {
		m_spinnerKraken.setVelocity(desiredVelocity, Kraken.VelocityControlType.VelocityVoltage, 1);
	}
	public void resetSpinnerEncoder() {
		m_spinnerKraken.motor.setPosition(0);
	}
}
package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.controls.DutyCycleOut;

import edu.wpi.first.wpilibj.DigitalInput;
import frc.util.Kraken;

public class ClimberKraken {
	private static ClimberKraken instance = null;
	public DigitalInput beambreak;


	public Kraken m_climberKraken = new Kraken(ClimberConstants.kClimbermotorID, ClimberConstants.ClimberConfigs
		.climberWheelConfigs(), "Climber", "ClimberMotor");
	public Kraken m_spinnerKraken = new Kraken(ClimberConstants.kClimberspinnerID, ClimberConstants.ClimberConfigs
		.climberWheelConfigs(), "Climber", "SpinnerMotor");

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

	public void runManual(double percentOutputElbow, double percentOutputWrist) {
		m_climberKraken.motor.setControl(new DutyCycleOut(percentOutputElbow));
		m_spinnerKraken.motor.setControl(new DutyCycleOut(percentOutputWrist));
	}

	public void runPosition(double desiredRotations) {
		m_climberKraken.setPosition(desiredRotations, Kraken.PositionControlType.PositionVoltage);
	}

	public void spinrunPosition(double desiredRotations) {
		m_spinnerKraken.setPosition(desiredRotations, Kraken.PositionControlType.PositionVoltage);
	}

	public void spinRunVelocity(double desiredVelocity) {
		m_spinnerKraken.setVelocity(desiredVelocity, Kraken.VelocityControlType.VelocityVoltage);
	}
}

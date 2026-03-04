package frc.robot.subsystems.Intake;

import frc.util.Kraken;
import frc.util.KrakenWithCancoder;

public class IntakeKraken {
	private KrakenWithCancoder m_ExtendKraken = new KrakenWithCancoder(IntakeConstants.kIntakeMotorIDExtend, IntakeConstants.kIntakeEncoderID, IntakeConstants.IntakeConfigs
		.IntakeExtendConfigs(), IntakeConstants.IntakeConfigs.IntakeEncoderConfigs(), "Intake", "Extend");
	private Kraken m_RollerKraken = new Kraken(IntakeConstants.kIntakeMotorIDRoller, IntakeConstants.IntakeConfigs
		.RollerConfigs(), "Intake", "Roller");

	private static IntakeKraken instance = null;

	public IntakeKraken() {
		m_ExtendKraken.setup();
		m_RollerKraken.setup();
	}

	public static IntakeKraken getInstance() {
		return instance = instance == null ? new IntakeKraken() : instance;
	}

	public void updateStats() {
		m_ExtendKraken.refresh();
		m_RollerKraken.refresh();
	}

	public double getExtendPosition() {
		return m_ExtendKraken.position.getValueAsDouble();
	}

	public double getRetractPosition() {
		return m_ExtendKraken.position.getValueAsDouble();
	}

	public void runPosition(double desiredRotations) {
		m_ExtendKraken.setPosition(desiredRotations, Kraken.PositionControlType.MotionMagicVoltage);
	}

	public void runVelocity(double desiredVelocity) {
		m_RollerKraken.setVelocity(desiredVelocity, Kraken.VelocityControlType.VelocityVoltage);
	}
	public void stop() {
		m_RollerKraken.stop();
	}
}

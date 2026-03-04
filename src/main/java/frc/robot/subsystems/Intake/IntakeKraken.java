package frc.robot.subsystems.Intake;

import frc.util.Kraken;
import frc.util.KrakenWithCancoder;

public class IntakeKraken {
	private KrakenWithCancoder m_ExtendKraken = new KrakenWithCancoder(IntakeConstants.kIntakeMotorIDExtend, IntakeConstants.kIntakeEncoderID, IntakeConstants.IntakeConfigs
		.IntakeExtendConfigs(), IntakeConstants.IntakeConfigs.IntakeEncoderConfigs(), "Intake", "Extend");
	private Kraken m_RollerKraken = new Kraken(IntakeConstants.kIntakeMotorIDRoller, IntakeConstants.IntakeConfigs
		.RollerConfigs(), "Intake", "Roller");

	private static IntakeKraken instance = null;

	private int controlVel = 0;
	private int controlPos = 0;

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
		switch (controlPos) {
			case 0:
				m_ExtendKraken.setPosition(desiredRotations, Kraken.PositionControlType.PositionVoltage, 0);
				break;
			case 1:
				m_ExtendKraken.setPosition(desiredRotations, Kraken.PositionControlType.PositionTorqueCurrentFOC, 1);
				break;
			case 2:
				m_ExtendKraken.setPosition(desiredRotations, Kraken.PositionControlType.MotionMagicExpoVoltage, 2);
				break;
			case 3:
				m_ExtendKraken.setPosition(desiredRotations, Kraken.PositionControlType.MotionMagicExpoTorqueCurrentFOC, 1);
				break;
		}
	}

	public void runVelocity(double RPM) {
		double desiredRotationsPerSecond = (RPM / 60.0);
		switch (controlVel) {
			case 0:
				m_RollerKraken.setVelocity(desiredRotationsPerSecond, Kraken.VelocityControlType.VelocityVoltage, 0);
				break;
			case 1:
				m_RollerKraken.setVelocity(desiredRotationsPerSecond, Kraken.VelocityControlType.VelocityTorqueCurrentFOC, 1);
				break;
		}
	}
	public void stop() {
		m_RollerKraken.stop();
	}

}

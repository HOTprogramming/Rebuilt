package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import frc.util.Kraken;
import frc.util.KrakenWithCancoder;

public class IntakeKraken {
	public KrakenWithCancoder m_ExtendKraken = new KrakenWithCancoder(IntakeConstants.kIntakeMotorIDExtend, IntakeConstants.kIntakeEncoderID, IntakeConstants.IntakeConfigs
		.IntakeExtendConfigs(), IntakeConstants.IntakeConfigs.IntakeEncoderConfigs(), "Intake", "Extend");
	public Kraken m_RollerKrakenLeader = new Kraken(IntakeConstants.kIntakeMotorIDRollerLeader, "rio", IntakeConstants.IntakeConfigs
		.RollerConfigs(), "Intake", "Roller");
	public Kraken m_RollerKrakenFollower = new Kraken(IntakeConstants.kIntakeMotorIDRollerFollower, "rio", IntakeConstants.IntakeConfigs
		.RollerConfigs(), "Intake", "Roller");

	private static IntakeKraken instance = null;

	public IntakeKraken() {
		m_ExtendKraken.setup();
		m_RollerKrakenLeader.setup();
		m_RollerKrakenFollower.setup();
		m_ExtendKraken.motor.getConfigurator().apply(IntakeConstants.IntakeExtendConfigs.MotionMagic);
		m_ExtendKraken.motor.getConfigurator().apply(new MotionMagicConfigs()
			.withMotionMagicAcceleration(64)
			.withMotionMagicCruiseVelocity(64));

		//m_RollerKrakenFollower.setFollower(IntakeConstants.kIntakeMotorIDRollerLeader, MotorAlignmentValue.Aligned);
	}

	public static IntakeKraken getInstance() {
		return instance = instance == null ? new IntakeKraken() : instance;
	}

	public void updateStats() {
		m_ExtendKraken.refresh();
		m_RollerKrakenLeader.refresh();
		m_RollerKrakenFollower.refresh();
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

	public void runAgitatePosition(double desiredRotations) {
		m_ExtendKraken.setPosition(desiredRotations, Kraken.PositionControlType.MotionMagicVoltage);
	}

	public void runVelocity(double desiredVelocity) {
		m_RollerKrakenLeader.setVelocity(desiredVelocity, Kraken.VelocityControlType.VelocityTorqueCurrentFOC);
		m_RollerKrakenFollower.setVelocity(desiredVelocity, Kraken.VelocityControlType.VelocityTorqueCurrentFOC);
	}
	public void stop() {
		m_RollerKrakenLeader.stop();
		m_RollerKrakenFollower.stop();
	}
}

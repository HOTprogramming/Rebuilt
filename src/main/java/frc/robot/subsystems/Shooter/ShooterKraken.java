package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.signals.MotorAlignmentValue;
import frc.util.Kraken;

public class ShooterKraken {
	private static ShooterKraken instance = null;

	private int controlModeShoot = 1;
	private int controlModeHood = 0;

	public Kraken m_krakenFXLeftTop = new Kraken(ShooterConstants.kMotorIDLeftTop, ShooterConstants.ShooterConfigs.shooterMotorConfigs(), "Shooter", "Left Top");
	public Kraken m_krakenFXLeftBottom = new Kraken(ShooterConstants.kMotorIDLeftBottom, ShooterConstants.ShooterConfigs.shooterMotorConfigs(), "Shooter", "Left Bottom");
	public Kraken m_krakenFXRightTop = new Kraken(ShooterConstants.kMotorIDRightTop, ShooterConstants.ShooterConfigs.shooterMotorConfigs(), "Shooter", "Right Top");

	public Kraken m_krakenFXHood = new Kraken(ShooterConstants.kHoodMotorID, ShooterConstants.ShooterConfigs.hoodMotorConfigs(), "Shooter", "Hood");

	public ShooterKraken() {
		m_krakenFXLeftTop.setup();
		m_krakenFXLeftBottom.setup();
		m_krakenFXRightTop.setup();
		m_krakenFXHood.setup();

		m_krakenFXLeftBottom.setFollower(ShooterConstants.kMotorIDLeftTop, MotorAlignmentValue.Aligned);
		m_krakenFXRightTop.setFollower(ShooterConstants.kMotorIDLeftTop, MotorAlignmentValue.Aligned);
	}

	public static ShooterKraken getInstance() {
		return instance = instance == null ? new ShooterKraken() : instance;
	}

	public void updateStats() {
		m_krakenFXLeftTop.refresh();
		m_krakenFXLeftBottom.refresh();
		m_krakenFXRightTop.refresh();
		m_krakenFXHood.refresh();
	}

	public void runVelocity(double desiredRotationsPerSecond) {
		switch (controlModeShoot) {
			case 0:
				m_krakenFXLeftTop.setVelocity(desiredRotationsPerSecond, Kraken.VelocityControlType.VelocityVoltage, 0);
				break;
			case 1:
				m_krakenFXLeftTop.setVelocity(desiredRotationsPerSecond, Kraken.VelocityControlType.VelocityTorqueCurrentFOC, 1);
		}
	}

	public void runPosition(double desiredRotations) {
		switch (controlModeHood) {
			case 0:
				m_krakenFXHood.setPosition(desiredRotations, Kraken.PositionControlType.PositionVoltage, 0);
				break;
			case 1:
				m_krakenFXHood.setPosition(desiredRotations, Kraken.PositionControlType.PositionTorqueCurrentFOC, 1);
		}
	}

	public void stop() {
		m_krakenFXLeftTop.stop();
		m_krakenFXLeftBottom.stop();
		m_krakenFXRightTop.stop();
	}
}
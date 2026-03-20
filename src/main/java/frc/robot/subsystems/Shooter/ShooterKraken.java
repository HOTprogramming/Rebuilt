package frc.robot.subsystems.shooter;

import frc.util.Kraken;

public class ShooterKraken {
	public Kraken m_krakenFXLeftTop = new Kraken(ShooterConstants.kMotorIDLeftTop, ShooterConstants.ShooterConfigs.shooterMotorConfigs(), "Shooter", "Left Top");
	public Kraken m_krakenFXLeftBottom = new Kraken(ShooterConstants.kMotorIDLeftBottom, ShooterConstants.ShooterConfigs.shooterMotorConfigs(), "Shooter", "Left Bottom");
	public Kraken m_krakenFXRightTop = new Kraken(ShooterConstants.kMotorIDRightTop, ShooterConstants.ShooterConfigs.shooterMotorConfigs(), "Shooter", "Right Top");
	public Kraken m_krakenFXHood = new Kraken(ShooterConstants.kHoodMotorID, ShooterConstants.ShooterConfigs.hoodMotorConfigs(), 
	"Shooter", "Hood");

	public ShooterKraken() {
		m_krakenFXLeftTop.setup();
		m_krakenFXLeftBottom.setup();
		m_krakenFXRightTop.setup();
		m_krakenFXHood.setup();

		m_krakenFXHood.motor.setPosition(0);
	}

	public void update() {
		m_krakenFXLeftTop.update();
		m_krakenFXLeftBottom.update();
		m_krakenFXRightTop.update();
		m_krakenFXHood.update();
	}

	public void runVelocity(double desiredRotationsPerSecond) {
		if(desiredRotationsPerSecond == 0) {
			stop();
			return;
		}
		m_krakenFXLeftTop.setVelocity(desiredRotationsPerSecond, Kraken.VelocityControlType.VelocityTorqueCurrentFOC, 1);
		m_krakenFXLeftBottom.setVelocity(desiredRotationsPerSecond, Kraken.VelocityControlType.VelocityTorqueCurrentFOC, 1);
		m_krakenFXRightTop.setVelocity(desiredRotationsPerSecond, Kraken.VelocityControlType.VelocityTorqueCurrentFOC, 1);
	}

	public void runPosition(double desiredRotations) {
		m_krakenFXHood.setPosition(desiredRotations, Kraken.PositionControlType.PositionVoltage, 0);
	}

	public void stop() {
		m_krakenFXLeftTop.stop();
		m_krakenFXLeftBottom.stop();
		m_krakenFXRightTop.stop();
	}
}
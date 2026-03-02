package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.Kraken;

public class Shooter extends SubsystemBase {
	private final ShooterKraken mShooterKraken = ShooterKraken.getInstance();

	public Shooter() {
		mShooterKraken.m_krakenFXHood.motor.setPosition(0.0);
	}

	public double getTargetShootSpeed() {
		return ShooterConstants.kTargetShootSpeed;
	}

	public double getTargetHoodAngle() {
		return ShooterConstants.kTargetHoodAngle;
	}

	public double getShooterSpeed() {
		return mShooterKraken.m_krakenFXLeftTop.velocity.getValueAsDouble();
	}

	@Override
	public void periodic() {
		mShooterKraken.updateStats();
	}

	public Command shooterOff() {
		return runOnce(() -> {
			mShooterKraken.stop();
		});
	}

	public Command shootSpeedUp() {
		return runOnce(() -> {
			ShooterConstants.kTargetShootSpeed += 0.5;
			mShooterKraken.runVelocity(ShooterConstants.kTargetShootSpeed);
		});
	}

	public Command shootSpeedDown() {
		return runOnce(() -> {
			ShooterConstants.kTargetShootSpeed -= 0.5;
			mShooterKraken.runVelocity(ShooterConstants.kTargetShootSpeed);
		});
	}

	public Boolean readyToShoot() {
		return (getShooterSpeed() >= ShooterConstants.kShooterSpeed - ShooterConstants.kShooterError);
	}

	public Command goToIdle() {
		return runOnce(() -> {
			mShooterKraken.runVelocity(ShooterConstants.kIdleSpeed);
		});
	}

	public Command goToShoot() {
		return runOnce(() -> {
			mShooterKraken.runVelocity(ShooterConstants.kShooterSpeed);
		});
	}

	public Command setHoodAngle(double angle) {
		return runOnce(() -> {
			mShooterKraken.runPosition(angle);
		});
	}

	public Command hoodAngleUp() {
		return runOnce(() -> {
			ShooterConstants.kTargetHoodAngle += 0.5;
			mShooterKraken.runPosition(ShooterConstants.kTargetHoodAngle);
		});
	}

	public Command hoodAngleDown() {
		return runOnce(() -> {
			ShooterConstants.kTargetHoodAngle -= 0.5;
			mShooterKraken.runPosition(ShooterConstants.kTargetHoodAngle);
		});
	}
}
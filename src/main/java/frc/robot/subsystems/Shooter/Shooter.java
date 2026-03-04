package frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.util.Kraken;

public class Shooter extends SubsystemBase {
	private final ShooterKraken mShooterKraken = ShooterKraken.getInstance();

	private InterpolatingDoubleTreeMap hoodAngleMap;
	private InterpolatingDoubleTreeMap shooterSpeedMap;


	public Shooter() {
		hoodAngleMap = new InterpolatingDoubleTreeMap();
		hoodAngleMap.put(1.0, 1.7);
		hoodAngleMap.put(2.0, 4.2);
		hoodAngleMap.put(3.0, 5.4);
		hoodAngleMap.put(4.0, 6.0);
		hoodAngleMap.put(5.0, 6.7);
		hoodAngleMap.put(6.0, 7.3);

		shooterSpeedMap = new InterpolatingDoubleTreeMap();
		shooterSpeedMap.put(1.0, 40.9);
		shooterSpeedMap.put(2.0, 41.9);
		shooterSpeedMap.put(3.0, 45.0);
		shooterSpeedMap.put(4.0, 47.9);
		shooterSpeedMap.put(5.0, 50.8);
		shooterSpeedMap.put(6.0, 53.7);

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

	public Command autoAimHub() {
		double distance = RobotState.pose.getTranslation().getDistance(DriverStation.getAlliance().get() == Alliance.Blue ? ShooterConstants.blueHub : ShooterConstants.redHub);
		return runOnce(() -> {
			mShooterKraken.runPosition(hoodAngleMap.get(distance));
		});
	}

	
	public Command autoAimLeftCorner() {
		double distance = RobotState.pose.getTranslation().getDistance(new Translation2d(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 16.54, 8.07));
		return runOnce(() -> {
			mShooterKraken.runPosition(hoodAngleMap.get(distance));
		});
	}

	public Command autoAimRightCorner() {
		double distance = RobotState.pose.getTranslation().getDistance(new Translation2d(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 16.54, 0));
		return runOnce(() -> {
			mShooterKraken.runPosition(hoodAngleMap.get(distance));
		});
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
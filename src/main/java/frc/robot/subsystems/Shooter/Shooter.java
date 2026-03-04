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
		hoodAngleMap.put(7.0, 7.9);
				hoodAngleMap.put(8.0, 8.5);
		hoodAngleMap.put(9.0, 9.2);
		hoodAngleMap.put(10.0, 9.8);
		hoodAngleMap.put(11.0, 10.4);
		hoodAngleMap.put(12.0, 11.0);
		hoodAngleMap.put(13.0, 11.6);



		shooterSpeedMap = new InterpolatingDoubleTreeMap();
		shooterSpeedMap.put(1.0, 40.9);
		shooterSpeedMap.put(2.0, 41.9);
		shooterSpeedMap.put(3.0, 45.0);
		shooterSpeedMap.put(4.0, 47.9);
		shooterSpeedMap.put(5.0, 50.8);
		shooterSpeedMap.put(6.0, 53.7);
		shooterSpeedMap.put(7.0, 56.7);
				shooterSpeedMap.put(8.0, 59.6);
		shooterSpeedMap.put(9.0, 61.5);
		shooterSpeedMap.put(10.0, 65.4);
		shooterSpeedMap.put(11.0, 68.3);
		shooterSpeedMap.put(12.0, 71.2);
		shooterSpeedMap.put(13.0, 74.1);




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
		return runOnce(() -> {
			double distance = RobotState.pose.getTranslation().getDistance(DriverStation.getAlliance().get() == Alliance.Blue ? ShooterConstants.blueHub : ShooterConstants.redHub);
			mShooterKraken.runPosition(hoodAngleMap.get(distance));
			mShooterKraken.runVelocity(shooterSpeedMap.get(distance));

		});
	}

	
	public Command autoAimLeftCorner() {
		return runOnce(() -> {
			double distance = RobotState.pose.getTranslation().getDistance(new Translation2d(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 16.54, 8.07));
			mShooterKraken.runPosition(hoodAngleMap.get(distance));
			mShooterKraken.runVelocity(shooterSpeedMap.get(distance));

		});
	}

	public Command autoAimRightCorner() {
		return runOnce(() -> {
			double distance = RobotState.pose.getTranslation().getDistance(new Translation2d(DriverStation.getAlliance().get() == Alliance.Blue ? 0 : 16.54, 0));
			mShooterKraken.runPosition(hoodAngleMap.get(distance));
			mShooterKraken.runVelocity(shooterSpeedMap.get(distance));
		});
	}

	public Command dodgeTrench() {
		return runOnce(() -> {
			mShooterKraken.runPosition(0);
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
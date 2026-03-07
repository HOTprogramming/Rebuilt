package frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

		// hoodAngleMap.put(1.0, 2.0); 
		// hoodAngleMap.put(2.0, 3.8);
		// hoodAngleMap.put(3.0, 5.4);
		// hoodAngleMap.put(4.0, 6.0);
		// hoodAngleMap.put(5.0, 6.7);

		hoodAngleMap.put(1.0, 0.027); // 0.027
		hoodAngleMap.put(2.0, 0.047); // 0.047
		hoodAngleMap.put(3.0, 0.065); // 0.065
		hoodAngleMap.put(4.0, 0.073); // 0.073
		hoodAngleMap.put(5.0, 0.088); // 0.088

		shooterSpeedMap = new InterpolatingDoubleTreeMap();

		shooterSpeedMap.put(1.0, 41.5);
		shooterSpeedMap.put(2.0, 41.9);
		shooterSpeedMap.put(3.0, 43.0);
		shooterSpeedMap.put(4.0, 46.5);
		shooterSpeedMap.put(5.0, 50.0);

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

	private void shooterHub() {
		mShooterKraken.runVelocity(shooterSpeedMap.get(RobotState.lookAheadDistance));
	}

	private void shooterLeftCorner() {
		double distance = RobotState.pose.getTranslation().getDistance(new Translation2d(RobotState.color == Alliance.Blue ? 0 : 16.54, 8.07));
		mShooterKraken.runVelocity(shooterSpeedMap.get(distance));
	}

	private void shooterRightCorner() {
		double distance = RobotState.pose.getTranslation().getDistance(new Translation2d(RobotState.color == Alliance.Blue ? 0 : 16.54, 0));
		mShooterKraken.runVelocity(shooterSpeedMap.get(distance));
	}

	private void hoodHub() {
		mShooterKraken.runPosition(hoodAngleMap.get(RobotState.lookAheadDistance));
	}

	private void hoodLeftCorner() {
		double distance = RobotState.pose.getTranslation().getDistance(new Translation2d(RobotState.color == Alliance.Blue ? 0 : 16.54, 8.07));
		mShooterKraken.runPosition(hoodAngleMap.get(distance));
	}

	private void hoodRightCorner() {
		double distance = RobotState.pose.getTranslation().getDistance(new Translation2d(RobotState.color == Alliance.Blue ? 0 : 16.54, 0));
		mShooterKraken.runPosition(hoodAngleMap.get(distance));
	}


	public Command autoAimHub() {
		return runOnce(() -> {
			shooterHub();
		});
	}
	public Command autoAimRightCorner() {
		return runOnce(() -> {
			shooterRightCorner();
		});
	}

	public Command autoAimLeftCorner() {
		return runOnce(() -> {
			shooterLeftCorner();
		});
	}

		public Command autoAimHubwithhood() {
		return runOnce(() -> {
			shooterHub();
			hoodHub();
		});
	}
	public Command autoAimRightCornerwithhood() {
		return runOnce(() -> {
			shooterRightCorner();
			hoodRightCorner();
		});
	}

	public Command autoAimLeftCornerwithhood() {
		return runOnce(() -> {
			shooterLeftCorner();
			hoodLeftCorner();
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

	public Command presetTrench() {
		return runOnce(() -> {
			mShooterKraken.runPosition(hoodAngleMap.get(3.0));	
			mShooterKraken.runVelocity(shooterSpeedMap.get(3.0));
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

	public BooleanSupplier readyToShoot() {
		return () -> (getShooterSpeed() >= ShooterConstants.kShooterSpeed - ShooterConstants.kShooterError && (getTargetHoodAngle() >= ShooterConstants.kTargetHoodAngle));
	}

}
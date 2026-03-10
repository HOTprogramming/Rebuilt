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

	double targetSpeed = 0;
	double targetHood = 0;

	double distance = 0;

	public Shooter() {
		hoodAngleMap = new InterpolatingDoubleTreeMap();

		hoodAngleMap.put(1.0, 0.027);
		hoodAngleMap.put(2.0, 0.047);
		hoodAngleMap.put(3.0, 0.065);
		hoodAngleMap.put(4.0, 0.073);
		hoodAngleMap.put(5.0, 0.088);
		hoodAngleMap.put(5.1, 0.105);
		hoodAngleMap.put(17.0, 0.145);


		shooterSpeedMap = new InterpolatingDoubleTreeMap();

		shooterSpeedMap.put(1.0, 41.5);
		shooterSpeedMap.put(2.0, 42.5);
		shooterSpeedMap.put(3.0, 44.0);
		shooterSpeedMap.put(4.0, 46.5);
		shooterSpeedMap.put(5.0, 50.0);
		shooterSpeedMap.put(5.1, 46.0);
		shooterSpeedMap.put(17.0, 90.0);





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
		double distance = RobotState.pose.getTranslation().getDistance(RobotState.color == Alliance.Blue ? ShooterConstants.blueHub : ShooterConstants.redHub);
		SmartDashboard.putNumber("distance to hub", distance);
		SmartDashboard.putBoolean("Shooter Reached Point", readyToShoot());
	}

	public Command autoAim() {
		return run(() -> {
			targetHood = hoodAngleMap.get(RobotState.lookAheadDistance);
			targetSpeed = shooterSpeedMap.get(RobotState.lookAheadDistance);
			mShooterKraken.runPosition(targetHood);
			mShooterKraken.runVelocity(targetSpeed);
		});
	}

	public Command maintainFlywheel() {
		return run(() -> {
			targetSpeed = shooterSpeedMap.get(RobotState.lookAheadDistance);
			mShooterKraken.runVelocity(targetSpeed);
			mShooterKraken.runPosition(0);
		});
	}

	public Command idle() {
		return run(() -> {
			mShooterKraken.runVelocity(ShooterConstants.kIdleSpeed);
			mShooterKraken.runPosition(0);
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

	public Command PresetBatter() {
		return runOnce(() -> {
			mShooterKraken.runPosition(0);
			mShooterKraken.runVelocity(0); 
		});
	}

	public Command PresetTrench() {
		return runOnce(() -> {
			mShooterKraken.runPosition(0);
			mShooterKraken.runVelocity(0); 
		});
	}

	public Command PresetClimberPole() {
		return runOnce(() -> {
			mShooterKraken.runPosition(0);
			mShooterKraken.runVelocity(0); 
		});
	}

	public boolean readyToShoot() {
		return (
			Math.abs(getShooterSpeed() - targetSpeed) <= ShooterConstants.kShooterError &&
			Math.abs(mShooterKraken.m_krakenFXHood.position.getValueAsDouble() - targetHood) <= ShooterConstants.kShooterHoodError);
	}

}
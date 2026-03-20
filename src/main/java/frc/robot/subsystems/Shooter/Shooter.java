package frc.robot.subsystems.shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;

public class Shooter extends SubsystemBase {
	public final ShooterKraken mShooterKraken = new ShooterKraken();

	private InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
	private InterpolatingDoubleTreeMap shooterSpeedMap =  new InterpolatingDoubleTreeMap();

	public double targetSpeed = 0;
	public double targetHood = 0;

	public double shotMapOffset = 0;

	public Shooter() {
		hoodAngleMap.put(1.0, 2.7); 
		hoodAngleMap.put(2.0, 3.6);
		hoodAngleMap.put(3.0, 5.0);
		hoodAngleMap.put(4.0, 5.916);
		hoodAngleMap.put(5.3, 6.196);
		// Passing
		hoodAngleMap.put(5.35, 9.66); 
		hoodAngleMap.put(17.0, 10.24);

		shooterSpeedMap.put(1.0, 41.0);
		shooterSpeedMap.put(2.0, 42.0);
		shooterSpeedMap.put(3.0, 45.0);
		shooterSpeedMap.put(4.0, 47.5);
		shooterSpeedMap.put(5.3, 53.0);
		//Passing
		shooterSpeedMap.put(5.35, 46.0);
		shooterSpeedMap.put(17.0, 90.0);
	}

	public double getShooterSpeed() {
		return mShooterKraken.m_krakenFXLeftTop.motor.getVelocity().getValueAsDouble();
	}

	public double getHoodAngle() {
		return mShooterKraken.m_krakenFXHood.motor.getPosition().getValueAsDouble();
	}

	@Override
	public void periodic() {
		mShooterKraken.update();
	}

	public Command hoodDown(){
		return runOnce(() -> {
			mShooterKraken.runPosition(0);
			mShooterKraken.runVelocity(targetSpeed);
		});
	}

	public Command autoAim() {
		return run(() -> {
			targetHood = hoodAngleMap.get(RobotState.lookAheadDistance + shotMapOffset);
			targetSpeed = shooterSpeedMap.get(RobotState.lookAheadDistance + shotMapOffset);
			mShooterKraken.runPosition(targetHood);
			mShooterKraken.runVelocity(targetSpeed);
		});
	}

	public Command PresetBatter() {
		return runOnce(() -> {
			targetHood = 2.8;			
			targetSpeed = 41.0;
			mShooterKraken.runPosition(targetHood);
			mShooterKraken.runVelocity(targetSpeed); 
		});
	}

	public Command PresetTrench() {
		return runOnce(() -> {
			targetHood = hoodAngleMap.get(3.6);
			targetSpeed = shooterSpeedMap.get(3.6);
			mShooterKraken.runPosition(targetHood);
			mShooterKraken.runVelocity(targetSpeed);
		});
	}

	public Command PresetClimberPole() {
		return runOnce(() -> {
			targetHood = 6.0;
			targetSpeed = 45.0;
			mShooterKraken.runPosition(targetHood);
			mShooterKraken.runVelocity(targetSpeed); 
		});
	}

	public boolean readyToShoot() {
		return (
			Math.abs(getShooterSpeed() - targetSpeed) <= ShooterConstants.kShooterError &&
			Math.abs(getHoodAngle() - targetHood) <= ShooterConstants.kShooterHoodError);
	}
}
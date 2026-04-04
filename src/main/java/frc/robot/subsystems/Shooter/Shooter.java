package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.Zone;

public class Shooter extends SubsystemBase {
	public final ShooterKraken mShooterKraken = new ShooterKraken();

	private InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
	private InterpolatingDoubleTreeMap shooterSpeedMap =  new InterpolatingDoubleTreeMap();

	public double targetSpeed = 0;
	public double targetHood = 0;

	public double shotMapOffset = -0.15;

	public Shooter() {
		hoodAngleMap.put(1.0, 1.0); 
		hoodAngleMap.put(1.5, 2.5);
		hoodAngleMap.put(2.0, 2.85);
		hoodAngleMap.put(3.0, 4.0);
		hoodAngleMap.put(4.0, 5.0);
		hoodAngleMap.put(5.3, 7.0);
		// Passing
		hoodAngleMap.put(5.35, 7.5); 
		hoodAngleMap.put(17.0, 7.5);

		shooterSpeedMap.put(1.0, 41.0);
		shooterSpeedMap.put(2.0, 42.0);
		shooterSpeedMap.put(3.0, 43.5);
		shooterSpeedMap.put(4.0, 45.0);
		shooterSpeedMap.put(5.3, 47.5);
		//Passing
		shooterSpeedMap.put(5.35, 41.0);
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
		SmartDashboard.putNumber("target hood", targetHood);		
		SmartDashboard.putNumber("target speed", targetSpeed);

	}

	public Command hoodDown(){
		return run(() -> {
			RobotState.isShooting = false;
			if(!RobotState.isManualMode()){
				targetHood = 0;
				mShooterKraken.runPosition(targetHood);
			} else {
				mShooterKraken.runPosition(0);
			}

			if(targetSpeed != 0){
				if(!RobotState.isManualMode()){
					if(RobotState.getZone() == Zone.ALLIANCE) targetSpeed = shooterSpeedMap.get(RobotState.lookAheadDistance + shotMapOffset);
					else targetSpeed = ShooterConstants.kIdleSpeed;
				}
				mShooterKraken.runVelocity(targetSpeed);
			} else {
				mShooterKraken.runVelocity(targetSpeed);
			}
		});
	}

	public Command shotmapTuning() {
		return run(() -> {
			mShooterKraken.runPosition(targetHood);
			mShooterKraken.runVelocity(targetSpeed);
		});
	}


	public Command autoAim() {
		return run(() -> {
			RobotState.isShooting = true;
			if(!RobotState.isManualMode()){
				if(DriverStation.isAutonomous()){
					double distance = RobotState.pose.getTranslation().getDistance(RobotState.target);
					targetHood = hoodAngleMap.get(distance);
					targetSpeed = shooterSpeedMap.get(distance);
				} else {
					targetHood = hoodAngleMap.get(RobotState.lookAheadDistance + shotMapOffset);
					targetSpeed = shooterSpeedMap.get(RobotState.lookAheadDistance + shotMapOffset);
				}
			}
			mShooterKraken.runPosition(targetHood);
			mShooterKraken.runVelocity(targetSpeed);
		});
	}

	public Command PresetBatter() {
		return runOnce(() -> {
			RobotState.setManualMode();
			targetHood = 1.5; // 2.8			
			targetSpeed = 41;
			mShooterKraken.runPosition(targetHood);
			mShooterKraken.runVelocity(targetSpeed); 
		});
	}

	public Command PresetTrench() {
		return runOnce(() -> {
			RobotState.setManualMode();
			targetHood = 3.2;
			targetSpeed = 43.0;
			mShooterKraken.runPosition(targetHood);
			mShooterKraken.runVelocity(targetSpeed);
		});
	}

	public Command PresetClimberPole() {
		return runOnce(() -> {
			RobotState.setManualMode();
			targetHood = 5.7; // 6.0
			targetSpeed = 45.0;
			mShooterKraken.runPosition(targetHood);
			mShooterKraken.runVelocity(targetSpeed); 
		});
	}

	public Command stop(){
		return runOnce(() -> {
			targetSpeed = 0;
			mShooterKraken.stop();
		});
	}

	public boolean readyToShoot() {
		return (
			Math.abs(getShooterSpeed() - targetSpeed) <= ShooterConstants.kShooterError &&
			Math.abs(getHoodAngle() - targetHood) <= ShooterConstants.kShooterHoodError);
	}
}
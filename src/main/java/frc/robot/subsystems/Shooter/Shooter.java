package frc.robot.subsystems.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
<<<<<<< HEAD
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
=======
>>>>>>> a721243b7ca975bde9cfcef03f69f24138f110a9
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.util.Kraken;

public class Shooter extends SubsystemBase {
	public final ShooterKraken mShooterKraken = ShooterKraken.getInstance();

	private InterpolatingDoubleTreeMap hoodAngleMap;
	private InterpolatingDoubleTreeMap shooterSpeedMap;

	public double targetSpeed = 0;
	public double targetHood = 0;

	public double shotMapOffset = 0;

	double distance = 0;

	public Shooter() {
		hoodAngleMap = new InterpolatingDoubleTreeMap();

		hoodAngleMap.put(1.0, 2.7); 
		hoodAngleMap.put(2.0, 3.6); //4.3
		hoodAngleMap.put(3.0, 5.0); //6.1  5.9
		hoodAngleMap.put(4.0, 5.916);
		hoodAngleMap.put(5.3, 6.196);

		hoodAngleMap.put(5.35, 9.66);
		hoodAngleMap.put(17.0, 10.24);



		shooterSpeedMap = new InterpolatingDoubleTreeMap();

		shooterSpeedMap.put(1.0, 41.0);
		shooterSpeedMap.put(2.0, 42.0);
		shooterSpeedMap.put(3.0, 45.0);
		shooterSpeedMap.put(4.0, 47.5);
		shooterSpeedMap.put(5.3, 53.0);

		shooterSpeedMap.put(5.35, 46.0);
		shooterSpeedMap.put(17.0, 90.0);
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
<<<<<<< HEAD
		mShooterKraken.updateStats();
		double distance = RobotState.pose.getTranslation().getDistance(RobotState.color == Alliance.Blue ? ShooterConstants.blueHub : ShooterConstants.redHub);
		SmartDashboard.putNumber("distance to hub", distance);
		SmartDashboard.putBoolean("Shooter Reached Point", readyToShoot());
=======
		mShooterKraken.update();
	}

	public Command hoodDown(){
		return run(() -> {
			RobotState.isShooting = false;
			if(RobotState.isManualMode()){
				RobotState.setNormalMode();
			}
			targetHood = 0;
			mShooterKraken.runPosition(targetHood);
			if(targetSpeed != 0){
				if(RobotState.getZone() == Zone.ALLIANCE) targetSpeed = shooterSpeedMap.get(RobotState.lookAheadDistance + shotMapOffset);
				else targetSpeed = ShooterConstants.kIdleSpeed;
				mShooterKraken.runVelocity(targetSpeed);
			} else {
				mShooterKraken.runVelocity(targetSpeed);
			}
		});
>>>>>>> a721243b7ca975bde9cfcef03f69f24138f110a9
	}

	public Command autoAim() {
		return run(() -> {
			RobotState.isShooting = true;
			if(DriverStation.isAutonomous()){
				double distance = RobotState.pose.getTranslation().getDistance(RobotState.target);
				targetHood = hoodAngleMap.get(distance);
				targetSpeed = shooterSpeedMap.get(distance);
			} else {
				targetHood = hoodAngleMap.get(RobotState.lookAheadDistance + shotMapOffset);
				targetSpeed = shooterSpeedMap.get(RobotState.lookAheadDistance + shotMapOffset);
			}
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
			targetSpeed = shooterSpeedMap.get(RobotState.lookAheadDistance);
			if(RobotState.mode == RobotState.Mode.normal) {
				mShooterKraken.runVelocity(targetSpeed);
			}
		});
	}

	public Command holdTarget() {
		return runOnce(() -> {
			mShooterKraken.runPosition(targetHood);
			mShooterKraken.runVelocity(targetSpeed);
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
			Math.abs(mShooterKraken.m_krakenFXHood.position.getValueAsDouble() - targetHood) <= ShooterConstants.kShooterHoodError);
	}

	
	public Command AutonSpin() {
		return runOnce(() -> {
			targetSpeed = 42.0;
			mShooterKraken.runVelocity(targetSpeed); 
		});
	}

}
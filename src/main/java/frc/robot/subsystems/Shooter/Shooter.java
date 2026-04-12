package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import frc.robot.RobotState.Zone;
import frc.robot.subsystems.Indexer.Index;

public class Shooter extends SubsystemBase {
	public final ShooterKraken mShooterKraken = new ShooterKraken();

	private InterpolatingDoubleTreeMap hoodAngleMap = new InterpolatingDoubleTreeMap();
	private InterpolatingDoubleTreeMap shooterSpeedMap =  new InterpolatingDoubleTreeMap();
	// Per-distance max hood offset (degrees) when kicker wheels are bogged down.
	// Close shots barely need compensation; far shots are much more sensitive
	// to reduced ball feed speed because exit velocity matters more at range.
	// These values represent the MAX offset at full deficit — the actual offset
	// is scaled by how far below threshold the kicker speed has dropped.
	// Tune these at the field by watching shot accuracy at each distance
	// with a full vs. empty hopper.
	private InterpolatingDoubleTreeMap hoodKickerOffsetMap = new InterpolatingDoubleTreeMap();

	public double targetSpeed = 0;
	public double targetHood = 0;

	public double shotMapOffset = -0.15;

	// Reference to the indexer so we can read kicker wheel speed for hood offset
	private Index indexer;

	public Shooter(Index indexer) {
		this.indexer = indexer;
		hoodAngleMap.put(1.0, 1.0); 
		hoodAngleMap.put(1.5, 2.5);
		hoodAngleMap.put(2.0, 2.85);
		hoodAngleMap.put(3.0, 4.0);
		hoodAngleMap.put(4.0, 5.0);
		hoodAngleMap.put(5.3, 7.0);
		// Passing
		hoodAngleMap.put(5.35, 7.5); 
		hoodAngleMap.put(17.0, 11.0);

		shooterSpeedMap.put(1.0, 41.0);
		shooterSpeedMap.put(2.0, 42.0);
		shooterSpeedMap.put(3.0, 43.5);
		shooterSpeedMap.put(4.0, 45.0);
		shooterSpeedMap.put(5.3, 47.5);
		//Passing
		shooterSpeedMap.put(5.35, 41.0);
		shooterSpeedMap.put(17.0, 95.0);

		hoodKickerOffsetMap.put(1.0, 0.0);
		hoodKickerOffsetMap.put(2.0, 0.25);
		hoodKickerOffsetMap.put(3.0, 0.5);
		hoodKickerOffsetMap.put(4.0, 0.75);
		hoodKickerOffsetMap.put(5.3, 1.0);
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
		//SmartDashboard.putNumber("target hood", targetHood);		
		//SmartDashboard.putNumber("target speed", targetSpeed);

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
				double distance;
				if(DriverStation.isAutonomous()){
					distance = RobotState.pose.getTranslation().getDistance(RobotState.target);
				} else {
					distance = RobotState.lookAheadDistance + shotMapOffset;
				}

				targetHood = hoodAngleMap.get(distance);
				targetSpeed = shooterSpeedMap.get(distance);

				// Kicker-speed hood offset: when kicker wheels are slower than expected
				// (full hopper pressure, shoot-on-the-move ball loading), raise the hood
				// to compensate for the lower ball exit velocity. The offset is proportional
				// to how far below the threshold the kicker speed has dropped, scaled by
				// distance (far shots need more compensation) and the overall gain knob.
				// See ShooterConstants for tuning guide.
				if (ShooterConstants.kHoodOffsetEnabled) {
					double kickerRatio = indexer.getAverageKickerSpeed() / ShooterConstants.kKickerNominalSpeed;
					if (kickerRatio < ShooterConstants.kKickerSpeedThreshold) {
						double deficit = (ShooterConstants.kKickerSpeedThreshold - kickerRatio) / ShooterConstants.kKickerSpeedThreshold;
						targetHood += hoodKickerOffsetMap.get(distance) * deficit * ShooterConstants.kHoodOffsetScale;
					}
				}
			}
			mShooterKraken.runPosition(targetHood);
			mShooterKraken.runVelocity(targetSpeed);
		});
	}

	public Command PresetBatter() { // 1m
		return runOnce(() -> {
			RobotState.setManualMode();
			targetHood = hoodAngleMap.get(1.0); // 2.8			
			targetSpeed = shooterSpeedMap.get(1.0);
			mShooterKraken.runPosition(hoodAngleMap.get(1.0));
			mShooterKraken.runVelocity(shooterSpeedMap.get(1.0)); 
		});
	}

	public Command PresetTrench() { // 3.6m
		return runOnce(() -> {
			RobotState.setManualMode();
			targetHood = hoodAngleMap.get(3.5);
			targetSpeed = shooterSpeedMap.get(3.5);
			mShooterKraken.runPosition(hoodAngleMap.get(3.5));
			mShooterKraken.runVelocity(shooterSpeedMap.get(3.5));
		});
	}


	public Command PresetClimberPole() { // 2.8m
		return runOnce(() -> {
			RobotState.setManualMode();
			targetHood = hoodAngleMap.get(2.8); // 6.0
			targetSpeed = shooterSpeedMap.get(2.8);
			mShooterKraken.runPosition(hoodAngleMap.get(2.8));
			mShooterKraken.runVelocity(shooterSpeedMap.get(2.8)); 
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
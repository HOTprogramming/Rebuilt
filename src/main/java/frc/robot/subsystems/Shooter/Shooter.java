package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

	public double shotMapOffset = 0.0;


	private NetworkTableEntry entryTuningMode;
	private NetworkTableEntry entryHoodAngle;
	private NetworkTableEntry entryWheelSpeed; 

	// Reference to the indexer so we can read kicker wheel speed for hood offset
	private Index indexer;

	// Tracks kicker speed between loop cycles to compute rate of change (rps/sec).
	// Used by the anticipation term in the hood offset — if kicker is decelerating
	// (e.g. balls entering hopper), we increase the offset before the speed actually
	// drops below threshold. If kicker is accelerating (recovering), we back off early.
	private double prevKickerSpeed = 0;
	private double prevKickerTimestamp = 0;

	// Timer for the initial shot burst — adds a small fixed hood bump for the first
	// kShotStartDuration seconds after the indexer fires. The very first ball always
	// arrives slower because the kicker hasn't reached steady-state feeding yet,
	// and the kicker speed reading won't reflect this fast enough.
	private Timer shotStartTimer = new Timer();
	private boolean shotStartActive = false;

	public Shooter(Index indexer) {
		this.indexer = indexer;

		hoodAngleMap.put(1.0, 1.0); 

		hoodAngleMap.put(2.0, 3.7); //3.7
		hoodAngleMap.put(2.5, 4.7);
		hoodAngleMap.put(3.0, 5.2); //5.5
		hoodAngleMap.put(4.0, 6.9); //6.9
		hoodAngleMap.put(5.0, 8.2); //8.4
		hoodAngleMap.put(5.3, 8.7); //8.7
		// Passing
		hoodAngleMap.put(5.35, 7.5); 
		hoodAngleMap.put(17.0, 11.0); //11.0

		shooterSpeedMap.put(1.0, 38.0);

		shooterSpeedMap.put(2.0, 39.0);
		shooterSpeedMap.put(2.5, 40.6);
		shooterSpeedMap.put(3.0, 41.5); //41.0
		shooterSpeedMap.put(4.0, 44.1); //43.7
		shooterSpeedMap.put(5.0, 47.0);
		shooterSpeedMap.put(5.3, 48.5);
		//Passing
		shooterSpeedMap.put(5.35, 41.0);
		shooterSpeedMap.put(17.0, 95.0);

		// Mods to support shotmap tuning using Advantage Scope 
		//Get the default instance of Network table 
		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		//Next get Shooter Table 
		NetworkTable table = inst.getTable("HOT/Shooter/Shotmap");
        
		//Setup fields for update from Advantage Scope 
		entryTuningMode = table.getEntry("Shotmap Tuning Mode");
		entryHoodAngle = table.getEntry("Hood Rotations");
		entryWheelSpeed = table.getEntry("Wheel Speed");
		
		// Write initial values
        entryHoodAngle.setDouble(0.00);
        entryWheelSpeed.setDouble(0.00); 
		entryTuningMode.setBoolean(false);
		//****End of Shotmap Mods  */


		hoodKickerOffsetMap.put(1.0, 0.0); 
		hoodKickerOffsetMap.put(2.0, 0.25); 
		hoodKickerOffsetMap.put(3.0, 0.5); 

		hoodKickerOffsetMap.put(4.0, 0.75); 
		hoodKickerOffsetMap.put(5.3, 0.1); 
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
		// SmartDashboard.putNumber("target hood", targetHood);		
		// SmartDashboard.putNumber("target speed", targetSpeed);

	}

	public Command reverseShooter() {
		return Commands.sequence(
			runOnce(() -> mShooterKraken.runVelocity(0.1)),
			Commands.waitSeconds(0.5),
			run(() -> mShooterKraken.runVelocity(-20)));
	}

	/**
	 * Direct method to start the shot timer — called from indexOnCommand() in the
	 * same runOnce as the kicker start so they fire on the exact same loop cycle.
	 */
	public void startShotTimerNow() {
		shotStartTimer.restart();
		shotStartActive = true;
	}

	public Command hoodDown(){
		return run(() -> {
			RobotState.isShooting = false;
			// Reset kicker tracking so the anticipation term doesn't carry stale
			// data into the next shot (e.g. a big speed delta from the previous release)
			prevKickerSpeed = 0;
			prevKickerTimestamp = 0;
			shotStartActive = false;
			if(!RobotState.isManualMode()){
				targetHood = 0;
				mShooterKraken.runPosition(targetHood);
			} else {
				mShooterKraken.runPosition(0);
			}

			if(targetSpeed != 0){
				if(!RobotState.isManualMode()){
					if(RobotState.getZone() == Zone.ALLIANCE) targetSpeed = shooterSpeedMap.get(RobotState.lookAheadDistance + shotMapOffset);
					else targetSpeed = shooterSpeedMap.get(RobotState.lookAheadDistance + shotMapOffset) * ShooterConstants.kIdleSpeed;
				}
				mShooterKraken.runVelocity(targetSpeed);
			} else {
				mShooterKraken.runVelocity(targetSpeed);
			}
		});
	}

	public Command autoAim() {
		return run(() -> {

			RobotState.isShooting = true;
			if (!RobotState.isManualMode()) {
				double distance;
				if (DriverStation.isAutonomous()) {
					distance = RobotState.pose.getTranslation().getDistance(RobotState.target);
				} else {
					distance = RobotState.lookAheadDistance + shotMapOffset;
				}

				targetHood = hoodAngleMap.get(distance);
				targetSpeed = shooterSpeedMap.get(distance);
			}


			//Mods to support shotmap tuning within Advantage Scope
			boolean boolTuningMode = entryTuningMode.getBoolean(false);
			//If user has turned on tuning mode by setting Advantage Scope boolean to true 
			if(boolTuningMode) {
				//get the values from Advantage Scope
				targetHood = entryHoodAngle.getDouble(0.0);
				targetSpeed = entryWheelSpeed.getDouble(0.0);

				System.out.println("--- Changing Hood value to: "+targetHood);
				System.out.println("--- Changing Wheel Speed to: "+targetSpeed);
			}
			//End of Shotmap Mods 

			mShooterKraken.runPosition(targetHood);
			mShooterKraken.runVelocity(targetSpeed);
		}).withName("auto aim teleop");
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
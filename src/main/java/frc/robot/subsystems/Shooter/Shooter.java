package frc.robot.subsystems.Shooter;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
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

	private NetworkTableEntry entryTuningMode;
	private NetworkTableEntry entryHoodAngle;
	private NetworkTableEntry entryWheelSpeed; 

	public Shooter() {
		hoodAngleMap.put(1.0, 1.0); 

		hoodAngleMap.put(2.0, 3.7);
		hoodAngleMap.put(3.0, 5.5); //5.1
		hoodAngleMap.put(4.0, 7.1);
		hoodAngleMap.put(5.0, 8.4);
		hoodAngleMap.put(5.3, 8.7);
		// Passing
		hoodAngleMap.put(5.35, 7.5); 
		hoodAngleMap.put(17.0, 11.0);

		shooterSpeedMap.put(1.0, 38.0);

		shooterSpeedMap.put(2.0, 39.0);
		shooterSpeedMap.put(3.0, 41.8); //41.2
		shooterSpeedMap.put(4.0, 43.8); //43.3
		shooterSpeedMap.put(5.0, 45.8);

		shooterSpeedMap.put(5.3, 50.5);
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
		}).withName("auto aim");
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
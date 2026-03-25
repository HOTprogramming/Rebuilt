package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.configs.ToFParamsConfigs;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.signals.UpdateModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Timer;

import edu.wpi.first.wpilibj.DigitalInput;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import frc.util.Kraken;
import frc.util.KrakenWithCancoder;




public class ClimberKraken {
	private static ClimberKraken instance = null;
	public DigitalInput beambreak;

	private final CANBus kCANBus = CANBus.roboRIO();

	public CANrange canRange = new CANrange(50, kCANBus);	

	private static final double PRINT_PERIOD = 0.5; // Update every 500 ms
	
	public Kraken m_climberKraken = new KrakenWithCancoder(ClimberConstants.kClimbermotorID, ClimberConstants.kClimbEncoder, ClimberConstants.ClimberConfigs
		.elbowConfigs(), ClimberConstants.encoderConfigs, "Climber", "ClimberMotor");
	public Kraken m_spinnerKraken = new Kraken(ClimberConstants.kClimberspinnerID, ClimberConstants.ClimberConfigs
		.wristConfigs(), "Climber", "SpinnerMotor");

	public void robotInit() {

		CANrangeConfiguration config = new CANrangeConfiguration();

		config.ProximityParams.MinSignalStrengthForValidMeasurement = 2000;
	
		config.ProximityParams.ProximityThreshold = 0.1; // If CANrange detects an object within 0.1 meters, it will trigger the "isDetected" signal.
	
		config.ToFParams = new ToFParamsConfigs()
			.withUpdateMode(UpdateModeValue.ShortRangeUserFreq)
			.withUpdateFrequency(200.0);
		
		canRange.getConfigurator().apply(config);
	}
	
	private double currentTime = Timer.getFPGATimestamp();


	
	public ClimberKraken() {
		m_climberKraken.setup();
		m_spinnerKraken.setup();
		beambreak = new DigitalInput(0);
	}

	public static ClimberKraken getInstance() {
		return instance = instance == null ? new ClimberKraken() : instance;
	}

	public void updateStats() {
		m_climberKraken.update();
		m_spinnerKraken.update();
	}

	public boolean getBeambreakStatus() {
		return beambreak.get();
	}
	public double getWristPos() {
		return m_spinnerKraken.motor.getPosition().getValueAsDouble();
	}
	public double getElbowPos() {
		return m_climberKraken.motor.getPosition().getValueAsDouble();
	}

	public void runManual(double percentOutputElbow, double percentOutputWrist) {
		m_climberKraken.motor.setControl(new DutyCycleOut(percentOutputElbow * 0.3));
		m_spinnerKraken.motor.setControl(new DutyCycleOut(percentOutputWrist * 0.3));
	}

	public void setElbowPosition(double desiredRotations) {
		m_climberKraken.setPosition(desiredRotations, Kraken.PositionControlType.MotionMagicVoltage, 0);
	}
	public void setElbowVelocity(double desiredRotations) {
		m_climberKraken.setPosition(desiredRotations, Kraken.PositionControlType.MotionMagicVoltage, 1);
	}
	public void setWristPosition(double desiredRotations) {
		m_spinnerKraken.setPosition(desiredRotations, Kraken.PositionControlType.MotionMagicVoltage, 0);
	}

	public void setWristVelocity(double desiredVelocity) {
		m_spinnerKraken.setVelocity(desiredVelocity, Kraken.VelocityControlType.VelocityVoltage, 1);
	}
	public void resetWristEncoder() {
		m_spinnerKraken.motor.setPosition(0);
	}

	public void stop(){
		m_climberKraken.stop();
		m_spinnerKraken.stop();
	}
}
package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

public class ClimberConstants {
	public static int kClimbermotorID = 16;
	public static int kClimberspinnerID = 17;
	public static int kClimbEncoder = 44;

	public static String kBusName = Constants.canName;
	public static double storedPos = -0.2517;
	public static double deployedPos = 0.2761;
	public static double climbedPos = 0.152;
	public static double wristZeroOffset = (6.0/360.0) * 634.8;
	public static double wristGearRatio = 634.8;
	public static double wristPastClimbPos = 0.508;
	public static double wristBeforeClimbPos = 0.492;
	public static double wristClimbPos = 0.50;

	public static double liftClimbPos = 0.0555;
	public static double l3ElbowPos = 0.272;

	public static final CANcoderConfiguration encoderConfigs = new CANcoderConfiguration()
		.withMagnetSensor(new MagnetSensorConfigs()
			.withAbsoluteSensorDiscontinuityPoint(0.5)
			.withMagnetOffset(.117676)
			.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive));


	public static final TalonFXConfiguration elbowConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			.withStatorCurrentLimit(80)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(60)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(30)
			.withSupplyCurrentLowerTime(1))
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(40)
			.withPeakReverseTorqueCurrent(-40))
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Brake))
		.withMotionMagic(new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(10.0)
			.withMotionMagicAcceleration(10.0))
		.withFeedback(
			new FeedbackConfigs()
				.withFeedbackRemoteSensorID(kClimbEncoder)
				.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
				.withSensorToMechanismRatio(1.0)
				.withRotorToSensorRatio(1.0))
		.withSlot0(new Slot0Configs()
			.withKP(500)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(0.27)
			.withKV(1))
		.withSlot1(new Slot1Configs()
			.withKP(5.0)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(2.5))
		.withSlot2(new Slot2Configs()
			.withKS(1)
			.withKV(.12)
			.withKA(0.01)
			.withKP(5)
			.withKI(0)
			.withKD(0.1))
		.withMotionMagic(new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(80.0)
			.withMotionMagicAcceleration(160)
			.withMotionMagicJerk(1600)
		);

	public static final TalonFXConfiguration wristConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			.withStatorCurrentLimit(80)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(60)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(30)
			.withSupplyCurrentLowerTime(1))
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(40)
			.withPeakReverseTorqueCurrent(-40))
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Brake))
		.withMotionMagic(new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(10.0)
			.withMotionMagicAcceleration(10.0))
		.withSlot0(new Slot0Configs()
			.withKP(5)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(0.27)
			.withKV(3))
		.withSlot1(new Slot1Configs()
			.withKP(0.7)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(0.35)
			.withKV(0.08))
		.withSlot2(new Slot2Configs()
			.withKS(1)
			.withKV(0)
			.withKA(0)
			.withKP(5)
			.withKI(0)
			.withKD(0))
		.withMotionMagic(new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(80.0)
			.withMotionMagicAcceleration(160)
			.withMotionMagicJerk(1600)
		);


	/********************************************************
	 * Based on the robot selected, load Intake configs with the selected robot
	 * configuration
	 ********************************************************/
	public static final ClimberConfigs ClimberConfigs = switch (Constants.getRobot()) {
		case COMPBOT -> new ClimberConfigs(elbowConfigs, wristConfigs);
		case CAMERABOT -> new ClimberConfigs(elbowConfigs, wristConfigs);
		case COMP24 -> new ClimberConfigs(elbowConfigs, wristConfigs);
		case COMP25 -> new ClimberConfigs(elbowConfigs, wristConfigs);
		case MINIBOT -> new ClimberConfigs(elbowConfigs, wristConfigs);
		case ALPHABOT -> new ClimberConfigs(elbowConfigs, wristConfigs);
	};

	/*****************
	 * Define the Climber Config Class
	 *****************/
	public record ClimberConfigs(
		TalonFXConfiguration elbowConfigs, TalonFXConfiguration wristConfigs) {}

}
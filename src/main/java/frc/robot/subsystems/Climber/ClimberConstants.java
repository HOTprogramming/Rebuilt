package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
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
import frc.robot.Constants.RobotType;

public class ClimberConstants {
	public static int kClimbermotorID = Constants.getRobot() == RobotType.COMPBOT ? 20 : 16;
	public static int kClimberspinnerID = Constants.getRobot() == RobotType.COMPBOT ? 21 : 17;
	public static int kClimbEncoder = 47;

	public static String kBusName = Constants.canName;

	public static double wristGearRatio = 634.8;
	public static double elbowGearRatio = 602.18;

	public static double wristZeroOffset = (0.0/360.0) * wristGearRatio;
	public static double startL3ExtendAtPos = 0.3;
	public static double wristPastClimbPos = 0.508;
	public static double wristFinalAdjustPos = 0.492;
	public static double wristClimbPos = 0.50;

	public static double storedPos = 0;
	public static double deployedPos = -283.7 / elbowGearRatio;
	public static double l1autonclimbedpos = 0.438; // unset
	public static double startWristAtPos = 0.175;
	public static double liftElbowClimbPos = 0.438;
	public static double l3ElbowPos = 0.272;
	public static double wheelsUpPos = 0.2125;


	public static double liftSpeed = 0.06 * elbowGearRatio;


	public static final CANcoderConfiguration encoderConfigs = new CANcoderConfiguration()
		.withMagnetSensor(new MagnetSensorConfigs()
			.withAbsoluteSensorDiscontinuityPoint(0.5)
			.withMagnetOffset(-0.413)
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
				.withSensorToMechanismRatio(602.18))
		.withSlot0(new Slot0Configs()
			.withKP(500)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(0.27)
			.withKV(1))
		.withSlot1(new Slot1Configs()
			.withKP(300)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(0.27)
			.withKV(0.2))
		.withMotionMagic(new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(0.3)
			.withMotionMagicAcceleration(0.6)
			.withMotionMagicJerk(6)
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
			.withKV(0.5))
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
			.withMotionMagicCruiseVelocity(120.0)
			.withMotionMagicAcceleration(200)
			.withMotionMagicJerk(2000)
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

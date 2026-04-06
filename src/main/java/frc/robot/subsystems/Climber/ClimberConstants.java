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
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;
import frc.robot.Constants.RobotType;

public class ClimberConstants {
	public static int kClimbermotorID = Constants.getRobot() == RobotType.COMPBOT ? 20 : 16;
	public static int kClimberspinnerID = Constants.getRobot() == RobotType.COMPBOT ? 21 : 17;

	public static String kBusName = Constants.canName;

	public static double wristGearRatio = 634.8;
	public static double elbowGearRatio = 370.57;

	public static double wristZeroOffset = (0.0/360.0) * wristGearRatio;


	public static double startElbowAdjustmentPos = 0.15;
	public static double wristClimbPos = 0.515;

	public static double wristPastClimbPos = 0.508;
	public static double wristFinalAdjustPos = 0.492;


	public static double storedPos = 0;
	public static double deployedPos = -0.471;
	public static double initalElbowClimbPos = -0.705; // 0.675
	public static double adjustedElbowClimbPos = -0.72;
	public static double l3ElbowPos = -0.477;
	public static double autoClimbPos = -0.625; 

	public static double wheelsUpPos = 0.2125; // unset
	public static double l1autonclimbedpos = 0.438; // unset
	public static double startWristAtPos = -0.605;


	public static double liftSpeed = 0.06 * elbowGearRatio;


	public static final TalonFXConfiguration elbowConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			.withStatorCurrentLimit(160)
			.withStatorCurrentLimitEnable(false)
			.withSupplyCurrentLimit(160)
			.withSupplyCurrentLimitEnable(false)
			.withSupplyCurrentLowerLimit(140)
			.withSupplyCurrentLowerTime(1))
		// .withTorqueCurrent(new TorqueCurrentConfigs()
		// 	.withPeakForwardTorqueCurrent(160)
		// 	.withPeakReverseTorqueCurrent(-160))
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Brake))
		.withMotionMagic(new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(10.0)
			.withMotionMagicAcceleration(10.0))
		.withFeedback(
			new FeedbackConfigs()
				.withSensorToMechanismRatio(elbowGearRatio))
		.withSlot0(new Slot0Configs()
			.withKP(1000)
			.withKI(0.0)
			.withKD(6)
			.withKS(4)
			.withKV(0.1))
		.withSlot1(new Slot1Configs()
			.withKP(300)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(0.27)
			.withKV(0.2))
		.withMotionMagic(new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(2)
			.withMotionMagicAcceleration(2)
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
			.withMotionMagicCruiseVelocity(100.0)
			.withMotionMagicAcceleration(160.0)
			.withMotionMagicJerk(1000))
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
			.withKD(0));




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

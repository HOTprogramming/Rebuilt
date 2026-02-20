package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ClimberConstants {
	public static int kClimbermotorID = 16;
	public static int kClimberspinnerID = 17;
	public static String kBusName = Constants.canName;
	public static double deployPosition = 200.0;
	public static double retractPosition = 0.0;
	public static double spinPosition = 410;
	public static double spinhomePosition = 0;
	public static double lockPosition = 10;
	public static double lvlonePosition = 80;

	public static final TalonFXConfiguration climberWheelConfigs = new TalonFXConfiguration()
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
			.withKP(0.11)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(0.1)
			.withKV(0.12))
		.withSlot1(new Slot1Configs()
			.withKP(5.0)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(2.5))
		.withSlot2(new Slot2Configs()
			.withKS(0.25)
			.withKV(.12)
			.withKA(0.01)
			.withKP(4.8)
			.withKI(0)
			.withKD(0.1))
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
		case COMPBOT -> new ClimberConfigs(climberWheelConfigs);
		case CAMERABOT -> new ClimberConfigs(climberWheelConfigs);
		case COMP24 -> new ClimberConfigs(climberWheelConfigs);
		case COMP25 -> new ClimberConfigs(climberWheelConfigs);
		case MINIBOT -> new ClimberConfigs(climberWheelConfigs);
		case ALPHABOT -> new ClimberConfigs(climberWheelConfigs);
	};

	/*****************
	 * Define the Climber Config Class
	 *****************/
	public record ClimberConfigs(
		TalonFXConfiguration climberWheelConfigs) {}

}
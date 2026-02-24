package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class ShooterConstants {
	public static int kMotorIDLeftTop = 9;
	public static int kMotorIDLeftBottom = 10;
	public static int kMotorIDRightTop = 11;
	public static int kMotorIDRightBottom = 12;
	public static double kShootReduction = (1.0 / 1.0);
	public static double kIdleSpeed = (70.0);
	public static double kShooterSpeed = (32.0);
	public static double kTargetShootSpeed = kShooterSpeed;
	public static double kShooterError = (0.500);

	public static double kTargetHoodAngle = 0;
	public static int kHoodMotorID = 15;
	public static double kHoodReduction = (1.0 / 1.0);

	public static CANBus canbus = new CANBus(Constants.canName);

	public static final TalonFXConfiguration shooterMotorConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			.withStatorCurrentLimit(120)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(80)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(30)
			.withSupplyCurrentLowerTime(1))
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(80)
			.withPeakReverseTorqueCurrent(-80))
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.Clockwise_Positive)
			.withNeutralMode(NeutralModeValue.Coast))
		.withSlot0(new Slot0Configs()
			.withKP(2)
			.withKI(0.0)
			.withKD(0.05)
			.withKS(0.1)
			.withKV(0.3))
		.withSlot1(new Slot1Configs()
			.withKP(19)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(12)
			.withKV(.68));

	public static final TalonFXConfiguration hoodMotorConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			.withStatorCurrentLimit(80)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(40)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(30)
			.withSupplyCurrentLowerTime(1))
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(120)
			.withPeakReverseTorqueCurrent(-120))
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Coast))
		.withSlot0(new Slot0Configs()
			.withKP(2.5)
			.withKI(0.0)
			.withKD(0)
			.withKS(.1)
			.withKV(.1))
		.withSlot1(new Slot1Configs()
			.withKP(60.0)
			.withKI(0.0)
			.withKD(6.0));

	public static final ShooterConfigs ShooterConfigs = switch (Constants.getRobot()) {
		case COMPBOT -> new ShooterConfigs(shooterMotorConfigs, hoodMotorConfigs);
		case CAMERABOT -> new ShooterConfigs(shooterMotorConfigs, hoodMotorConfigs);
		case COMP24 -> new ShooterConfigs(shooterMotorConfigs, hoodMotorConfigs);
		case COMP25 -> new ShooterConfigs(shooterMotorConfigs, hoodMotorConfigs);
		case MINIBOT -> new ShooterConfigs(shooterMotorConfigs, hoodMotorConfigs);
		case ALPHABOT -> new ShooterConfigs(shooterMotorConfigs, hoodMotorConfigs);
	};

	public record ShooterConfigs(
		TalonFXConfiguration shooterMotorConfigs, TalonFXConfiguration hoodMotorConfigs) {}

}

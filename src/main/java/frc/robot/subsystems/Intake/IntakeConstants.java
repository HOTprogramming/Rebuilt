package frc.robot.subsystems.Intake;

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
import frc.robot.Robot;

public class IntakeConstants {
	public static double kDeployedPosition = 90;
	public static double kRetractPosition = 0.0;

	public static double kExpelSpeed = -2000.0;
	public static double kIntakeSpeed = 2000.0;

	public static int kIntakeEncoderID = 0;
	public static int kIntakeMotorIDExtend = 16;
	public static int kIntakeMotorIDRoller = 17;
	public static double kEncoderOffset = 0;
	public static String kBusName = Constants.canName;

	public static final TalonFXConfiguration RollerConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			.withStatorCurrentLimit(40)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(40)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(30)
			.withSupplyCurrentLowerTime(1))
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(40)
			.withPeakReverseTorqueCurrent(-40))
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Coast))
		.withMotionMagic(new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(10.0)
			.withMotionMagicAcceleration(10.0))
		.withSlot0(new Slot0Configs()
			.withKP(0.1)
			.withKI(0.1)
			.withKD(0.0)
			.withKS(0.4)
			.withKV(0.95))
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
			.withMotionMagicJerk(1600));

	public static final TalonFXConfiguration IntakeExtendConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			.withStatorCurrentLimit(40)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(40)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(30)
			.withSupplyCurrentLowerTime(1))
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(120)
			.withPeakReverseTorqueCurrent(-120))
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.Clockwise_Positive)
			.withNeutralMode(NeutralModeValue.Brake))
		.withSlot0(new Slot0Configs()
			.withKP(0.19)
			.withKI(0.0)
			.withKD(0.0))
		.withSlot1(new Slot1Configs()
			.withKP(60.0)
			.withKI(0.0)
			.withKD(6.0))
		.withSlot2(new Slot2Configs()
			.withKS(0.25)
			.withKV(.12)
			.withKA(0.01)
			.withKP(4.8)
			.withKI(0)
			.withKD(0.1))
		.withMotionMagic(new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(0.0)
			.withMotionMagicExpo_kV(0.12)
			.withMotionMagicExpo_kA(0.1))
		.withFeedback(new FeedbackConfigs()
			.withFeedbackRemoteSensorID(kIntakeEncoderID)
			.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
			.withSensorToMechanismRatio(1.0 / 360.0)
			.withRotorToSensorRatio(1.0));

	public static final CANcoderConfiguration IntakeEncoderConfigs = new CANcoderConfiguration()
		.withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(!Robot.isSimulation() ? kEncoderOffset : 0.0)
			.withSensorDirection(SensorDirectionValue.Clockwise_Positive));

	public static final IntakeConfigs IntakeConfigs = switch (Constants.getRobot()) {
		case COMPBOT -> new IntakeConfigs(RollerConfigs, IntakeExtendConfigs, IntakeEncoderConfigs);
		case CAMERABOT -> new IntakeConfigs(RollerConfigs, IntakeExtendConfigs, IntakeEncoderConfigs);
		case COMP24 -> new IntakeConfigs(RollerConfigs, IntakeExtendConfigs, IntakeEncoderConfigs);
		case COMP25 -> new IntakeConfigs(RollerConfigs, IntakeExtendConfigs, IntakeEncoderConfigs);
		case MINIBOT -> new IntakeConfigs(RollerConfigs, IntakeExtendConfigs, IntakeEncoderConfigs);
		case ALPHABOT -> new IntakeConfigs(RollerConfigs, IntakeExtendConfigs, IntakeEncoderConfigs);
	};

	public record IntakeConfigs(TalonFXConfiguration RollerConfigs, TalonFXConfiguration IntakeExtendConfigs,
		CANcoderConfiguration IntakeEncoderConfigs) {}
}

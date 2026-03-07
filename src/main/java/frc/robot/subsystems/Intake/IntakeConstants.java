package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.Robot;

public class IntakeConstants {
	public static double kDeployedPosition = 0.9;
	public static double kRetractPosition = 0.02;
	public static double kRetractPositionAgitate = 0.28;
	public static double kExtendTolerance = 0.01;
	public static double kRetractTolerance = 0.02;

	public static double kExpelSpeed = -100.0;
	public static double kIntakeSpeed = 200.0; //was 100

	public static int kIntakeEncoderID = Constants.getRobot() == RobotType.COMPBOT ? 44 : 0;
	public static int kIntakeMotorIDExtend = Constants.getRobot() == RobotType.COMPBOT ? 16 : 0;
	public static int kIntakeMotorIDRollerLeader = Constants.getRobot() == RobotType.COMPBOT ? 17 : 0;
	public static int kIntakeMotorIDRollerFollower = Constants.getRobot() == RobotType.COMPBOT ? 12 : 0;
	public static double kEncoderOffset = Constants.getRobot() == RobotType.COMPBOT ? -0.43 : 0.0;
	public static String kBusName = Constants.canName;

	public static final TalonFXConfiguration RollerConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			.withStatorCurrentLimit(60)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(40)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(30)
			.withSupplyCurrentLowerTime(1))
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(60)
			.withPeakReverseTorqueCurrent(-60))
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Coast))
		.withSlot0(new Slot0Configs()
			.withKP(1)
			.withKI(0)
			.withKD(0)
			.withKS(0)
			.withKV(0.12));

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
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Brake))
		.withSlot0(new Slot0Configs()
			.withKP(128)
			.withKI(0)
			.withKD(0))
		.withMotionMagic(new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(150.0)
			.withMotionMagicAcceleration(125.0))
		.withFeedback(new FeedbackConfigs()
			.withFeedbackRemoteSensorID(kIntakeEncoderID)
			.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
			.withSensorToMechanismRatio(1.0)
			.withRotorToSensorRatio(1.0));

	public static final CANcoderConfiguration IntakeEncoderConfigs = new CANcoderConfiguration()
		.withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(!Robot.isSimulation() ? kEncoderOffset : 0.0)
			.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive).withAbsoluteSensorDiscontinuityPoint(1));

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

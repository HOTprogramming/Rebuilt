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
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.Robot;

public class IntakeConstants {
	public static double kDeployedPosition = 0.07;
	public static double kShortDeployedPosition = 0.25; // not used

	public static double kRetractPosition = 0.61;
	public static double kRetractPositionAgitate = 0.55;
	public static double kExtendTolerance = 0.01;
	public static double kRetractTolerance = 0.01;

	public static double agitateCurrentThreshold = 20;
	public static double agitateCurrentTime = 0.2;

	public static double kExpelSpeed = -100.0;
	public static double kIntakeSpeed = 110.0;

	public static int kIntakeEncoderID = Constants.getRobot() == RobotType.COMPBOT ? 47 : 0;
	public static int kIntakeMotorIDExtend = Constants.getRobot() == RobotType.COMPBOT ? 16 : 0;
	public static int kIntakeMotorIDRollerLeader = Constants.getRobot() == RobotType.COMPBOT ? 17 : 0;
	public static int kIntakeMotorIDRollerFollower = Constants.getRobot() == RobotType.COMPBOT ? 12 : 0;
	public static int kIntakeMotorIDRoller3 = Constants.getRobot() == RobotType.COMPBOT ? 20 : 0;
	public static double kEncoderOffset = Constants.getRobot() == RobotType.COMPBOT ? -0.042 : 0.0;
	public static String kBusName = Constants.canName;

	public static final TalonFXConfiguration RollerConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			// .withStatorCurrentLimit(50)
			// .withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(50)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(30) //80
			.withSupplyCurrentLowerTime(0.5)) //was 2.0
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(50) //was 100
			.withPeakReverseTorqueCurrent(-50)) //was 100
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.Clockwise_Positive)
			.withNeutralMode(NeutralModeValue.Coast))
		.withSlot0(new Slot0Configs()
			.withKP(0.75)
			.withKI(0)
			.withKD(0)
			.withKS(0.35)
			.withKV(0.1));

		public static final TalonFXConfiguration autoRollerConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			// .withStatorCurrentLimit(50)
			// .withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(50)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(30) //80
			.withSupplyCurrentLowerTime(0.5)) //was 2.0
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(50) //was 100
			.withPeakReverseTorqueCurrent(-50)) //was 100
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.Clockwise_Positive)
			.withNeutralMode(NeutralModeValue.Coast))
		.withSlot0(new Slot0Configs()
			.withKP(0.75)
			.withKI(0)
			.withKD(0)
			.withKS(0.35)
			.withKV(0.1)); // 0.105


	public static final TalonFXConfiguration invertedRollerConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			// .withStatorCurrentLimit(50)
			// .withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(50)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(30) //80
			.withSupplyCurrentLowerTime(0.5)) //was 2.0
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(50) //was 100
			.withPeakReverseTorqueCurrent(-50)) //was 100
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Coast))
		.withSlot0(new Slot0Configs()
			.withKP(0.75)
			.withKI(0)
			.withKD(0)
			.withKS(0.35)
			.withKV(0.1));

		public static final TalonFXConfiguration invertedAutoRollerConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			// .withStatorCurrentLimit(50)
			// .withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(50)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(30) //80
			.withSupplyCurrentLowerTime(0.5)) //was 2.0
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(50) //was 100
			.withPeakReverseTorqueCurrent(-50)) //was 100
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Coast))
		.withSlot0(new Slot0Configs()
			.withKP(0.75)
			.withKI(0)
			.withKD(0)
			.withKS(0.35)
			.withKV(0.1)); // 0.105

	public static final TalonFXConfiguration IntakeExtendConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			.withStatorCurrentLimit(80)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(80)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(60)
			.withSupplyCurrentLowerTime(1))
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(80)
			.withPeakReverseTorqueCurrent(-80))
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Brake))
		.withSlot0(new Slot0Configs()
			.withKP(256) // 128
			.withKI(0)
			.withKD(0)
			.withKS(0.45)
			.withKV(0))
		.withMotionMagic(new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(16.0)
			.withMotionMagicAcceleration(32.0))
		.withFeedback(new FeedbackConfigs()	
			.withFeedbackRemoteSensorID(kIntakeEncoderID)
			.withFeedbackSensorSource(FeedbackSensorSourceValue.RemoteCANcoder)
			.withSensorToMechanismRatio(1));

	public static final CANcoderConfiguration IntakeEncoderConfigs = new CANcoderConfiguration()
		.withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(!Robot.isSimulation() ? kEncoderOffset : 0.0)
			.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive).withAbsoluteSensorDiscontinuityPoint(1));

	public static final IntakeConfigs IntakeConfigs = switch (Constants.getRobot()) {
		case COMPBOT -> new IntakeConfigs(RollerConfigs, IntakeExtendConfigs, autoRollerConfigs, invertedAutoRollerConfigs, invertedRollerConfigs, IntakeEncoderConfigs);
		case CAMERABOT -> new IntakeConfigs(RollerConfigs, IntakeExtendConfigs, autoRollerConfigs, invertedAutoRollerConfigs, invertedRollerConfigs, IntakeEncoderConfigs);
		case COMP24 -> new IntakeConfigs(RollerConfigs, IntakeExtendConfigs, autoRollerConfigs, invertedAutoRollerConfigs, invertedRollerConfigs, IntakeEncoderConfigs);
		case COMP25 -> new IntakeConfigs(RollerConfigs, IntakeExtendConfigs, autoRollerConfigs, invertedAutoRollerConfigs, invertedRollerConfigs, IntakeEncoderConfigs);
		case MINIBOT -> new IntakeConfigs(RollerConfigs, IntakeExtendConfigs, autoRollerConfigs, invertedAutoRollerConfigs, invertedRollerConfigs, IntakeEncoderConfigs);
		case ALPHABOT -> new IntakeConfigs(RollerConfigs, IntakeExtendConfigs, autoRollerConfigs, invertedAutoRollerConfigs, invertedRollerConfigs, IntakeEncoderConfigs);
	};

	public record IntakeConfigs(TalonFXConfiguration RollerConfigs, TalonFXConfiguration IntakeExtendConfigs, TalonFXConfiguration autonRollerConfigs, TalonFXConfiguration invertedAutoRollerConfigs, TalonFXConfiguration invertedRollerConfigs,
		CANcoderConfiguration IntakeEncoderConfigs) {}
}

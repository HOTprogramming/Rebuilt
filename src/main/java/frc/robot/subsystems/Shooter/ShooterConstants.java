package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;
import frc.robot.Robot;

public class ShooterConstants {
	public static int kMotorIDLeftTop = Constants.getRobot() == RobotType.COMPBOT ? 9 : 0;
	public static int kMotorIDLeftBottom = Constants.getRobot() == RobotType.COMPBOT ? 10 : 0;
	public static int kMotorIDRightTop = Constants.getRobot() == RobotType.COMPBOT ? 11 : 0;
	public static double kShootReduction = (1.0 / 1.0);
	public static double kIdleSpeed = (0.4); // percent of target speed
	public static double kShooterSpeed = (48.0);
	public static double kTargetShootSpeed = kShooterSpeed;
	public static double kShooterError = 2;
	public static double kShooterHoodError = .25;

	public static Translation2d redHub = new Translation2d(11.926, 4.048);
	public static Translation2d blueHub = new Translation2d(4.614, 4.048);

	private static Translation2d topBlueTrench = new Translation2d();
	private static Translation2d topRedTrench = new Translation2d();
	private static Translation2d bottomBlueTrench = new Translation2d();
	private static Translation2d bottomRedTrench = new Translation2d();

	public static Translation2d[] trenches = new Translation2d[] {topBlueTrench, topRedTrench, bottomBlueTrench, bottomRedTrench};

	public static double kTargetHoodAngle = 0;
	public static int kHoodMotorID = Constants.getRobot() == RobotType.COMPBOT ? 15 : 0;
	public static int kHoodEncoderID = Constants.getRobot() == RobotType.COMPBOT ? 45 : 0;
	public static double kHoodEncoderOffset = Constants.getRobot() == RobotType.COMPBOT ? .414 : 0.0;
	public static double kHoodReduction = (1.0 / 1.0);

	// --- Kicker-Speed Hood Offset ---
	// Dynamically raises the hood angle when kicker wheels are bogged down.
	// When the hopper is full, ball pressure slows the kicker wheels, reducing
	// the speed at which balls reach the flywheel. Slower ball feed means lower
	// exit velocity, so shots fall short. This also happens during shoot-on-the-move
	// when varying numbers of balls are being pulled through the indexer.
	//
	// The offset has two components:
	//
	// 1. PROPORTIONAL (deficit-based):
	//    Reacts to the current kicker speed relative to nominal.
	//    kickerRatio = actualSpeed / kKickerNominalSpeed
	//    If kickerRatio < kKickerSpeedThreshold, compute:
	//      deficit = (kKickerSpeedThreshold - kickerRatio) / kKickerSpeedThreshold
	//    Offset contribution = hoodKickerOffsetMap(distance) * deficit * kHoodOffsetScale
	//
	// 2. ANTICIPATION (derivative-based):
	//    Looks at the rate of change of kicker speed (acceleration in rps/sec).
	//    If the kicker is decelerating (balls pressing in, hopper loading), the hood
	//    starts rising BEFORE the speed actually drops below threshold.
	//    If the kicker is accelerating (recovering after balls clear), the hood starts
	//    coming back down early instead of waiting for the speed to fully recover.
	//    This prevents the hood from always lagging behind the kicker state.
	//    Offset contribution = -normalizedAccel * hoodKickerOffsetMap(distance) * kKickerAnticipationGain
	//
	// The final offset is clamped between 0 and the max offset for the current distance,
	// so it never subtracts from the base hood angle or exceeds the map maximum.
	//
	// Tuning guide:
	//   - kKickerNominalSpeed: expected kicker speed at full performance (rps).
	//     Start with the kicker command speed and lower if the wheels never actually reach it.
	//   - kKickerSpeedThreshold: ratio (0–1) below which the proportional term activates.
	//     0.95 means it kicks in when kicker drops below 95% of nominal.
	//     Lower = less sensitive, higher = activates sooner.
	//   - kHoodOffsetScale: gain on the proportional (deficit) term.
	//     1.0 = use the map values directly at full deficit. Increase if shots still fall short.
	//   - kKickerAnticipationGain: gain on the derivative (anticipation) term.
	//     Higher values = more aggressive anticipation. Start low (0.5) and increase
	//     if the hood is still lagging behind kicker speed changes. Too high will cause
	//     overshoot — the hood will overreact to brief speed fluctuations.
	//   - hoodKickerOffsetMap (in Shooter.java): per-distance max offset values.
	//     Close shots need little offset; far shots need more.
	public static boolean kHoodOffsetEnabled = false;
	public static double kKickerNominalSpeed = 110.0; // rps — matches IndexConstants.kickerSpeed
	public static double kKickerSpeedThreshold = 0.95; // ratio — proportional term activates below this
	public static double kHoodOffsetScale = 1.0; // gain on the proportional (deficit) term
	public static double kKickerAnticipationGain = 0.5; // gain on the derivative (anticipation) term

	public static CANBus canbus = new CANBus(Constants.canName);

	public static final TalonFXConfiguration shooterMotorConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			.withStatorCurrentLimit(80)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(60)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(40)
			.withSupplyCurrentLowerTime(1))
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(80)
			.withPeakReverseTorqueCurrent(-80))
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Coast))

		//SHOOTER PID
		.withSlot0(new Slot0Configs()
			.withKP(.55)
			.withKI(0.0)
			.withKD(0.001)
			.withKS(0.175)
			.withKV(0.115))
		.withSlot1(new Slot1Configs()
			.withKP(8)
			.withKI(0.0)
			.withKD(0)
			.withKS(5.5)
			.withKV(0.05));

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
			.withNeutralMode(NeutralModeValue.Brake))
		.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
			.withForwardSoftLimitThreshold(15)
			.withReverseSoftLimitThreshold(0)
			.withForwardSoftLimitEnable(true)
			.withReverseSoftLimitEnable(true))
			//HOOD PID
		.withSlot0(new Slot0Configs()
			.withKP(3.5)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(0.25)
			.withKV(0.0))
		.withSlot1(new Slot1Configs()
			.withKP(60.0)
			.withKI(0.0)
			.withKD(6.0))
		.withFeedback(new FeedbackConfigs());

	public static final CANcoderConfiguration hoodEncoderConfigs = new CANcoderConfiguration()
		.withMagnetSensor(new MagnetSensorConfigs().withMagnetOffset(!Robot.isSimulation() ? kHoodEncoderOffset : 0.0)
			.withSensorDirection(SensorDirectionValue.Clockwise_Positive).withAbsoluteSensorDiscontinuityPoint(0.5));

	public static final ShooterConfigs ShooterConfigs = switch (Constants.getRobot()) {
		case COMPBOT -> new ShooterConfigs(shooterMotorConfigs, hoodMotorConfigs, hoodEncoderConfigs);
		case CAMERABOT -> new ShooterConfigs(shooterMotorConfigs, hoodMotorConfigs, hoodEncoderConfigs);
		case COMP24 -> new ShooterConfigs(shooterMotorConfigs, hoodMotorConfigs, hoodEncoderConfigs);
		case COMP25 -> new ShooterConfigs(shooterMotorConfigs, hoodMotorConfigs, hoodEncoderConfigs);
		case MINIBOT -> new ShooterConfigs(shooterMotorConfigs, hoodMotorConfigs, hoodEncoderConfigs);
		case ALPHABOT -> new ShooterConfigs(shooterMotorConfigs, hoodMotorConfigs, hoodEncoderConfigs);
	};

	public record ShooterConfigs(
		TalonFXConfiguration shooterMotorConfigs, TalonFXConfiguration hoodMotorConfigs, CANcoderConfiguration hoodEncoderConfigs) {}

}
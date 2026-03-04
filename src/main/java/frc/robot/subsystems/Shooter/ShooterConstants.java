package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;

public class ShooterConstants {
	public static int kMotorIDLeftTop = Constants.getRobot() == RobotType.COMPBOT ? 9 : 0;
	public static int kMotorIDLeftBottom = Constants.getRobot() == RobotType.COMPBOT ? 10 : 0;
	public static int kMotorIDRightTop = Constants.getRobot() == RobotType.COMPBOT ? 11 : 0;
	public static double kShootReduction = (1.0 / 1.0);
	public static double kIdleSpeed = (10.0);
	public static double kShooterSpeed = (48.0);
	public static double kTargetShootSpeed = kShooterSpeed;
	public static double kShooterError = (0.500);

	public static Translation2d redHub = new Translation2d(11.926, 4.048);
	public static Translation2d blueHub = new Translation2d(4.614, 4.048);

	private static Translation2d topBlueTrench = new Translation2d();
	private static Translation2d topRedTrench = new Translation2d();
	private static Translation2d bottomBlueTrench = new Translation2d();
	private static Translation2d bottomRedTrench = new Translation2d();

	public static Translation2d[] trenches = new Translation2d[] {topBlueTrench, topRedTrench, bottomBlueTrench, bottomRedTrench};

	public static double kTargetHoodAngle = 0;
	public static int kHoodMotorID = Constants.getRobot() == RobotType.COMPBOT ? 15 : 0;
	public static double kHoodReduction = (1.0 / 1.0);

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
			.withInverted(InvertedValue.Clockwise_Positive)
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
			.withInverted(InvertedValue.Clockwise_Positive)
			.withNeutralMode(NeutralModeValue.Coast))
		.withSoftwareLimitSwitch(new SoftwareLimitSwitchConfigs()
			.withForwardSoftLimitThreshold(17)
			.withReverseSoftLimitThreshold(0)
			.withForwardSoftLimitEnable(true)
			.withReverseSoftLimitEnable(true))	

			//HOOD PID
		.withSlot0(new Slot0Configs()
			.withKP(3.5)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(0.5)
			.withKV(0.0))
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

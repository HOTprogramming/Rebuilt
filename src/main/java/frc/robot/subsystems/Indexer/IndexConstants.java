package frc.robot.subsystems.Indexer;

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
import frc.robot.Constants.RobotType;

public class IndexConstants {
	public static int kLeftKickID = Constants.getRobot() == RobotType.COMPBOT ? 18 : 14;
	public static int kRightKickID = Constants.getRobot() == RobotType.COMPBOT ? 19 : 15;
	public static int kFloorID = Constants.getRobot() == RobotType.COMPBOT ? 0 : 9; //13, set to 0 for broken floor
	public static int kLoaderID = Constants.getRobot() == RobotType.COMPBOT ? 0 : 13;//14, set to 0 for broken floor
	public static String kBusName = Constants.canName;
	public static double kickerSpeed = 100;
	public static double floorSpeed = 100;

	public static final TalonFXConfiguration indexWheelConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			.withStatorCurrentLimit(80)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(80)
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
			.withKP(0.3)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(0)
			.withKV(0.095))
		.withSlot1(new Slot1Configs()
			.withKP(0.25)
			.withKI(0.0)
			.withKD(0.0)
			.withKS(0)
			.withKV(0.12))
		.withMotionMagic(new MotionMagicConfigs()
			.withMotionMagicCruiseVelocity(80.0)
			.withMotionMagicAcceleration(160)
			.withMotionMagicJerk(1600)
		);

	/********************************************************
	 * Based on the robot selected, load Intake configs with the selected robot
	 * configuration
	 ********************************************************/
	public static final IndexConfigs IndexConfigs = switch (Constants.getRobot()) {
		case COMPBOT -> new IndexConfigs(indexWheelConfigs);
		case CAMERABOT -> new IndexConfigs(indexWheelConfigs);
		case COMP24 -> new IndexConfigs(indexWheelConfigs);
		case COMP25 -> new IndexConfigs(indexWheelConfigs);
		case MINIBOT -> new IndexConfigs(indexWheelConfigs);
		case ALPHABOT -> new IndexConfigs(indexWheelConfigs);
	};

	/*****************
	 * Define the Climber Config Class
	 *****************/
	public record IndexConfigs(
		TalonFXConfiguration indexWheelConfigs) {}

}

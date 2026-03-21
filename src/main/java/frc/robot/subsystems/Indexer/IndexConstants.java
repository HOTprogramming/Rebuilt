package frc.robot.subsystems.Indexer;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.Constants.RobotType;

public class IndexConstants {
	public static int kLeftKickID = Constants.getRobot() == RobotType.COMPBOT ? 18 : 14;
	public static int kRightKickID = Constants.getRobot() == RobotType.COMPBOT ? 19 : 15;
	public static int kFloorID = Constants.getRobot() == RobotType.COMPBOT ? 13 : 13;
	public static String kBusName = Constants.canName;
	public static double kickerSpeed = 120;
	public static double floorSpeed = 90;

	public static final TalonFXConfiguration indexWheelConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			.withStatorCurrentLimit(80)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(80)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(30)
			.withSupplyCurrentLowerTime(2))
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(80)
			.withPeakReverseTorqueCurrent(-80))
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Coast))
		.withSlot0(new Slot0Configs() //Wheels torque
			.withKP(120)
			.withKI(0.0)
			.withKD(0)
			.withKS(3)
			.withKV(2))
		.withSlot1(new Slot1Configs() //Roller Floor torque
			.withKP(240)
			.withKI(0.0)
			.withKD(0)
			.withKS(12)
			.withKV(1.6));


	public static final IndexConfigs IndexConfigs = switch (Constants.getRobot()) {
		case COMPBOT -> new IndexConfigs(indexWheelConfigs);
		case CAMERABOT -> new IndexConfigs(indexWheelConfigs);
		case COMP24 -> new IndexConfigs(indexWheelConfigs);
		case COMP25 -> new IndexConfigs(indexWheelConfigs);
		case MINIBOT -> new IndexConfigs(indexWheelConfigs);
		case ALPHABOT -> new IndexConfigs(indexWheelConfigs);
	};

	public record IndexConfigs(
		TalonFXConfiguration indexWheelConfigs) {}

}

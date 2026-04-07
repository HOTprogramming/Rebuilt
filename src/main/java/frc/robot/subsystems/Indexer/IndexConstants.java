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
	public static double kickerSpeed = 85;
	public static double floorSpeed = 70;

	public static final TalonFXConfiguration kickerConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			.withStatorCurrentLimit(80
			)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(80)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(60)
			.withSupplyCurrentLowerTime(1.5))
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(80)
			.withPeakReverseTorqueCurrent(-80))
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Coast))
		.withSlot0(new Slot0Configs() //Wheels torque
			.withKP(3)
			.withKI(0.0)
			.withKD(0)
			.withKS(5.1)
			.withKV(0.066)); // 0.066

		public static final TalonFXConfiguration floorConfigs = new TalonFXConfiguration()
		.withCurrentLimits(new CurrentLimitsConfigs()
			.withStatorCurrentLimit(80
			)
			.withStatorCurrentLimitEnable(true)
			.withSupplyCurrentLimit(80)
			.withSupplyCurrentLimitEnable(true)
			.withSupplyCurrentLowerLimit(60)
			.withSupplyCurrentLowerTime(1.5))
		.withTorqueCurrent(new TorqueCurrentConfigs()
			.withPeakForwardTorqueCurrent(80)
			.withPeakReverseTorqueCurrent(-80))
		.withMotorOutput(new MotorOutputConfigs()
			.withInverted(InvertedValue.CounterClockwise_Positive)
			.withNeutralMode(NeutralModeValue.Coast))
		.withSlot1(new Slot1Configs() //Roller Floor torque
			.withKP(15) //5
			.withKI(0.0)
			.withKD(0)
			.withKS(11.5) //11.5
			.withKV(0.1)); //0.1


	public static final IndexConfigs IndexConfigs = switch (Constants.getRobot()) {
		case COMPBOT -> new IndexConfigs(kickerConfigs, floorConfigs);
		case CAMERABOT -> new IndexConfigs(kickerConfigs, floorConfigs);
		case COMP24 -> new IndexConfigs(kickerConfigs, floorConfigs);
		case COMP25 -> new IndexConfigs(kickerConfigs, floorConfigs);
		case MINIBOT -> new IndexConfigs(kickerConfigs, floorConfigs);
		case ALPHABOT -> new IndexConfigs(kickerConfigs, floorConfigs);
	};

	public record IndexConfigs(
		TalonFXConfiguration kickerConfigs, TalonFXConfiguration floorConfigs) {}

}

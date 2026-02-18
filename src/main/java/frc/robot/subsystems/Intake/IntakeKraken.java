package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.sim.PhysicsSim;
import frc.robot.sim.PhysicsSimWithCan;

public class IntakeKraken {
	// Hardware
	private CANBus canBus = new CANBus(IntakeConstants.kBusName);
	private TalonFX m_Extend = new TalonFX(IntakeConstants.kIntakeMotorIDExtend, canBus);
	private TalonFX m_Roller = new TalonFX(IntakeConstants.kIntakeMotorIDRoller, canBus);
	private CANcoder m_CanCoder = new CANcoder(IntakeConstants.kIntakeEncoderID, canBus);

	// Control
	private final PositionVoltage m_positionVoltage = new PositionVoltage(0)
		.withSlot(0);
	private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0)
		.withSlot(1);
	private final MotionMagicExpoVoltage m_mmExpoVoltage = new MotionMagicExpoVoltage(0)
		.withSlot(2);
	private final MotionMagicExpoTorqueCurrentFOC m_mmExpoTorque = new MotionMagicExpoTorqueCurrentFOC(0.0)
		.withSlot(1);
	private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0)
		.withSlot(0);
	private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0)
		.withSlot(1);
	private final NeutralOut m_brake = new NeutralOut();

	private final StatusSignal<Angle> IntakePosition;
	private final StatusSignal<AngularVelocity> IntakeVelocity;
	private final StatusSignal<Voltage> IntakeAppliedVolts;
	private final StatusSignal<Current> IntakeSupplyCurrent;
	private final StatusSignal<Current> IntakeTorqueCurrent;
	private final StatusSignal<Temperature> IntakeTempCelsius;
	private final StatusSignal<Angle> IntakeEncoderPosition;
	private final StatusSignal<AngularVelocity> IntakeEncoderVelocity;

	private final StatusSignal<Angle> RollerPosition;
	private final StatusSignal<AngularVelocity> RollerVelocity;
	private final StatusSignal<Voltage> RollerAppliedVolts;
	private final StatusSignal<Current> RollerSupplyCurrent;
	private final StatusSignal<Current> RollerTorqueCurrent;
	private final StatusSignal<Temperature> RollerTempCelsius;

	private static IntakeKraken instance = null;

	private int controlVel = 0;
	private int controlPos = 0;
	private StatusCode statuscode = null;

	public IntakeKraken() {
		statuscode = m_Extend.getConfigurator().apply(IntakeConstants.IntakeConfigs.IntakeExtendConfigs(), 1.0);
		if (!statuscode.isOK()) {
			System.out.println("Configuration Failed on Left Extend shooter Motor ");
		}
		statuscode = m_Roller.getConfigurator().apply(IntakeConstants.IntakeConfigs.RollerConfigs(), 1.0);
		if (!statuscode.isOK()) {
			System.out.println("Configuration Failed on Left Roller shooter Motor ");
		}
		statuscode = m_CanCoder.getConfigurator().apply(IntakeConstants.IntakeConfigs.IntakeEncoderConfigs(), 1.0);
		if (!statuscode.isOK()) {
			System.out.println("Configuration Failed on LeftIntake CanCoder");
		}

		IntakePosition = m_Extend.getPosition();
		IntakeVelocity = m_Extend.getVelocity();
		IntakeAppliedVolts = m_Extend.getMotorVoltage();
		IntakeSupplyCurrent = m_Extend.getSupplyCurrent();
		IntakeTorqueCurrent = m_Extend.getTorqueCurrent();
		IntakeTempCelsius = m_Extend.getDeviceTemp();
		IntakeEncoderPosition = m_CanCoder.getPosition();
		IntakeEncoderVelocity = m_CanCoder.getVelocity();

		RollerPosition = m_Roller.getPosition();
		RollerVelocity = m_Roller.getVelocity();
		RollerAppliedVolts = m_Roller.getMotorVoltage();
		RollerSupplyCurrent = m_Roller.getSupplyCurrent();
		RollerTorqueCurrent = m_Roller.getTorqueCurrent();
		RollerTempCelsius = m_Roller.getDeviceTemp();

		BaseStatusSignal.setUpdateFrequencyForAll(100.0, IntakePosition, IntakeVelocity, IntakeAppliedVolts,
			IntakeSupplyCurrent, IntakeTorqueCurrent, IntakeTempCelsius, IntakeEncoderPosition,
			IntakeEncoderVelocity, RollerPosition, RollerVelocity, RollerAppliedVolts, RollerSupplyCurrent,
			RollerTorqueCurrent, RollerTempCelsius);

		m_Extend.optimizeBusUtilization();
		m_Roller.optimizeBusUtilization();

		if (Utils.isSimulation()) {
			PhysicsSimWithCan.getInstance().addTalonFX(m_Extend, m_CanCoder, 1, .001);
			PhysicsSim.getInstance().addTalonFX(m_Roller, 0.001);
		}
	}

	public static IntakeKraken getInstance() {
		return instance = instance == null ? new IntakeKraken() : instance;
	}

	public void updateStats(IntakeStats stats) {
		if (Utils.isSimulation()) {
			PhysicsSimWithCan.getInstance().run();
			PhysicsSim.getInstance().run();
		}

		stats.MotorConnected = BaseStatusSignal.refreshAll(IntakePosition, IntakeVelocity, IntakeAppliedVolts,
			IntakeSupplyCurrent, IntakeTorqueCurrent, IntakeTempCelsius, IntakeEncoderPosition,
			IntakeEncoderVelocity, RollerPosition, RollerVelocity, RollerAppliedVolts, RollerSupplyCurrent,
			RollerTorqueCurrent, RollerTempCelsius).isOK();

		stats.ExtendPositionRads = IntakePosition.getValueAsDouble();
		stats.ExtendVelocityRpm = IntakeVelocity.getValueAsDouble();
		stats.ExtendAppliedVolts = IntakeAppliedVolts.getValueAsDouble();
		stats.ExtendSupplyCurrentAmps = IntakeSupplyCurrent.getValueAsDouble();
		stats.ExtendTorqueCurrentAmps = IntakeTorqueCurrent.getValueAsDouble();
		stats.ExtendTempCelsius = IntakeTempCelsius.getValueAsDouble();
		stats.IntakeEncoderPosition = IntakeEncoderPosition.getValueAsDouble();
		stats.IntakeEncoderVelocity = IntakeEncoderVelocity.getValueAsDouble();
		stats.RollerPositionRads = RollerPosition.getValueAsDouble();
		stats.RollerVelocityRpm = RollerVelocity.getValueAsDouble();
		stats.RollerAppliedVolts = RollerAppliedVolts.getValueAsDouble();
		stats.RollerSupplyCurrentAmps = RollerSupplyCurrent.getValueAsDouble();
		stats.RollerTorqueCurrentAmps = RollerTorqueCurrent.getValueAsDouble();
		stats.RollerTempCelsius = RollerTempCelsius.getValueAsDouble();
	}

	public void runPosition(double desiredRotations) {
		switch (controlPos) {
			case 0:
				m_Extend.setControl(m_positionVoltage.withPosition(desiredRotations));
				break;
			case 1:
				m_Extend.setControl(m_positionTorque.withPosition(desiredRotations));
				break;
			case 2:
				m_Extend.setControl(m_mmExpoVoltage.withPosition(desiredRotations));
				break;
			case 3:
				m_Extend.setControl(m_mmExpoTorque.withPosition(desiredRotations));
				break;
		}
	}

	public void runVelocity(double RPM) {
		double desiredRotationsPerSecond = (RPM / 60.0);
		switch (controlVel) {
			case 0:
				m_Roller.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
				break;
			case 1:
				m_Roller.setControl(m_velocityTorque.withVelocity(desiredRotationsPerSecond));
				break;
		}
	}

	public void stop() {
		m_Roller.setControl(m_brake);
	}
}

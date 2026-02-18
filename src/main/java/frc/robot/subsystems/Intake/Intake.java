package frc.robot.subsystems.Intake;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
	private final IntakeKraken mIntakeKraken = IntakeKraken.getInstance();
	private final IntakeStats ioStats = new IntakeStats();

	private DoublePublisher ExtendPosition;
	private DoublePublisher ExtendVelocity;
	private DoublePublisher ExtendAppliedVolts;
	private DoublePublisher ExtendSupplyCurrent;
	private DoublePublisher ExtendTorqueCurrent;
	private DoublePublisher ExtendTemp;

	private DoublePublisher RollerPosition;
	private DoublePublisher RollerVelocity;
	private DoublePublisher RollerAppliedVolts;
	private DoublePublisher RollerSupplyCurrent;
	private DoublePublisher RollerTorqueCurrent;
	private DoublePublisher RollerTemp;

	public Intake() {
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTable table = inst.getTable("HOT/Intake");

		ExtendVelocity = table.getDoubleTopic("ExtendVelocity-Motor (rot per s)").publish();
		ExtendPosition = table.getDoubleTopic("ExtendPosition-Motor (rot)").publish();
		ExtendSupplyCurrent = table.getDoubleTopic("ExtendSupply Current").publish();
		ExtendTorqueCurrent = table.getDoubleTopic("ExtendTorque Current").publish();
		ExtendTemp = table.getDoubleTopic("ExtendMotor Temp").publish();
		ExtendAppliedVolts = table.getDoubleTopic("ExtendApplied Volts").publish();

		RollerVelocity = table.getDoubleTopic("RollerVelocity-Motor (rot per a)").publish();
		RollerPosition = table.getDoubleTopic("RollerPosition-Motor (rot)").publish();
		RollerSupplyCurrent = table.getDoubleTopic("RollerSupply Current").publish();
		RollerTorqueCurrent = table.getDoubleTopic("RollerTorque Current").publish();
		RollerTemp = table.getDoubleTopic("RollerMotor Temp").publish();
		RollerAppliedVolts = table.getDoubleTopic("RollerApplied Volts").publish();
	}

	@Override
	public void periodic() {
		mIntakeKraken.updateStats(ioStats);

		UpdateTelemetry();
	}

	private double roundTo1Decimal(double number) {
		return Math.round(number * 10.0) / 10.0;
	}

	private void UpdateTelemetry() {
		ExtendVelocity.set(roundTo1Decimal(ioStats.ExtendVelocityRpm));
		ExtendPosition.set(roundTo1Decimal(ioStats.ExtendPositionRads));
		ExtendSupplyCurrent.set(roundTo1Decimal(ioStats.ExtendSupplyCurrentAmps));
		ExtendTorqueCurrent.set(roundTo1Decimal(ioStats.ExtendTorqueCurrentAmps));
		ExtendAppliedVolts.set(roundTo1Decimal(ioStats.ExtendAppliedVolts));
		ExtendTemp.set(roundTo1Decimal(ioStats.ExtendTempCelsius));

		RollerVelocity.set(roundTo1Decimal(ioStats.RollerVelocityRpm));
		RollerPosition.set(roundTo1Decimal(ioStats.RollerPositionRads));
		RollerSupplyCurrent.set(roundTo1Decimal(ioStats.RollerSupplyCurrentAmps));
		RollerTorqueCurrent.set(roundTo1Decimal(ioStats.RollerTorqueCurrentAmps));
		RollerAppliedVolts.set(roundTo1Decimal(ioStats.RollerAppliedVolts));
		RollerTemp.set(roundTo1Decimal(ioStats.RollerTempCelsius));
	}

	public Command extend() {
		return runOnce(() -> {
			mIntakeKraken.runPosition(IntakeConstants.kDeployedPosition);
			mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
		});
	}

	public Command retract() {
		return runOnce(() -> {
			mIntakeKraken.runPosition(IntakeConstants.kRetractPosition);
			mIntakeKraken.stop();
		});
	}

	public Command expel() {
		return runOnce(() -> {
			mIntakeKraken.runPosition(IntakeConstants.kDeployedPosition);
			mIntakeKraken.runVelocity(IntakeConstants.kExpelSpeed);
		});
	}

	public Command stop() {
		return runOnce(() -> {
			mIntakeKraken.stop();
		});
	}
}

class IntakeStats {
	public boolean MotorConnected = true;
	public double ExtendPositionRads = 0.0;
	public double ExtendVelocityRpm = 0.0;
	public double ExtendAppliedVolts = 0.0;
	public double ExtendSupplyCurrentAmps = 0.0;
	public double ExtendTorqueCurrentAmps = 0.0;
	public double ExtendTempCelsius = 0.0;

	public double RollerPositionRads = 0.0;
	public double RollerVelocityRpm = 0.0;
	public double RollerAppliedVolts = 0.0;
	public double RollerSupplyCurrentAmps = 0.0;
	public double RollerTorqueCurrentAmps = 0.0;
	public double RollerTempCelsius = 0.0;
	public double IntakeEncoderVelocity;
	public double IntakeEncoderPosition;
}

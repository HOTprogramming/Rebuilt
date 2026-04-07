package frc.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.robot.Constants;
import frc.util.sim.PhysicsSim;

public class Kraken {

	public static enum PositionControlType{
		PositionVoltage,
		PositionTorqueCurrentFOC,
		MotionMagicVoltage,
		MotionMagicExpoVoltage,
		MotionMagicExpoTorqueCurrentFOC,
		MotionMagicTorqueCurrentFOC
	}

	public static enum VelocityControlType{
		VelocityVoltage,
		VelocityTorqueCurrentFOC
	}

	private final PositionVoltage positionVoltage = new PositionVoltage(0);
	private final PositionTorqueCurrentFOC positionTorque = new PositionTorqueCurrentFOC(0);
	private final MotionMagicVoltage mmVoltage = new MotionMagicVoltage(0);
	private final MotionMagicExpoVoltage mmExpoVoltage = new MotionMagicExpoVoltage(0);
	private final MotionMagicExpoTorqueCurrentFOC mmExpoTorque = new MotionMagicExpoTorqueCurrentFOC(0.0);
	private final VelocityVoltage velocityVoltage = new VelocityVoltage(0);
	private final VelocityTorqueCurrentFOC velocityTorque = new VelocityTorqueCurrentFOC(0);
	private final MotionMagicTorqueCurrentFOC mmTorque = new MotionMagicTorqueCurrentFOC(0);
	private final NeutralOut stop = new NeutralOut();

	public DoublePublisher positionPublisher;
	public DoublePublisher velocityPublisher;
	public DoublePublisher supplyCurrentPublisher;

	public final TalonFX motor;

	public final String subsystem;
	public final String deviceName;

	public Kraken(int id, TalonFXConfiguration config, InvertedValue inverted, String subsystem, String deviceName) {
		this(id, Constants.canName, config.withMotorOutput(config.MotorOutput.withInverted(inverted)), subsystem, deviceName);
	}

	public Kraken(int id, TalonFXConfiguration config, String subsystem, String deviceName) {
		this(id, Constants.canName, config, subsystem, deviceName);
	}

	public Kraken(int id, String canBusName, TalonFXConfiguration config, String subsystem, String deviceName) {
		motor = new TalonFX(id, new CANBus(canBusName));

		this.subsystem = subsystem;
		this.deviceName = deviceName;

		StatusCode statusCode = motor.getConfigurator().apply(config);

		if (!statusCode.isOK()) {
			System.out.println("Configuration Failed on motor " + subsystem + " " + deviceName);
		}
	}


	public void applyNewConfigs(TalonFXConfiguration config) {
		StatusCode statusCode = motor.getConfigurator().apply(config);

		if (!statusCode.isOK()) {
			System.out.println("Configuration Failed on motor " + subsystem + " " + deviceName);
		}
	}
	
	public void setup(){
		setupSim();

		createTelemetry(this.subsystem, this.deviceName);
	}

	public void setupSim(){
		PhysicsSim.getInstance().addTalonFX(motor, 0.001);
	}

	public void refreshSim(){
		PhysicsSim.getInstance().run();
	}

	public void update(){
		if (Utils.isSimulation()) {
			refreshSim();
		}

		updateTelemetry();
	}

	public void createTelemetry(String subsystem, String deviceName){
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("HOT/" + subsystem + "/" + deviceName);

        positionPublisher = table.getDoubleTopic(subsystem + deviceName + "Position").publish();
		velocityPublisher = table.getDoubleTopic(subsystem + deviceName + "Velocity").publish();
		supplyCurrentPublisher = table.getDoubleTopic(subsystem + deviceName + "Supply Current").publish();
	
	}

	public void updateTelemetry(){
		positionPublisher.set(motor.getPosition().getValueAsDouble());
		velocityPublisher.set(motor.getVelocity().getValueAsDouble());
		supplyCurrentPublisher.set(motor.getSupplyCurrent().getValueAsDouble());

	}

	public void stop() {
		motor.setControl(stop);
	}

	public void setPosition(double desiredRotations, PositionControlType controlType) {
		this.setPosition(desiredRotations, controlType, 0);
	}

	public void setPosition(double desiredRotations, PositionControlType controlType, int slot) {
		switch (controlType) {
			case PositionVoltage:
				motor.setControl(positionVoltage.withPosition(desiredRotations).withSlot(slot));
				break;
			case PositionTorqueCurrentFOC:
				motor.setControl(positionTorque.withPosition(desiredRotations).withSlot(slot));
				break;
			case MotionMagicVoltage:
				motor.setControl(mmVoltage.withPosition(desiredRotations).withSlot(slot));
				break;
			case MotionMagicExpoVoltage:
				motor.setControl(mmExpoVoltage.withPosition(desiredRotations).withSlot(slot));
				break;
			case MotionMagicExpoTorqueCurrentFOC:
				motor.setControl(mmExpoTorque.withPosition(desiredRotations).withSlot(slot));
				break;
			case MotionMagicTorqueCurrentFOC:
				motor.setControl(mmTorque.withPosition(desiredRotations).withSlot(slot));
				break;
		}
	}

	public void setVelocity(double speed, VelocityControlType controlType) {
		this.setVelocity(speed, controlType, 0);
	}

	public void setVelocity(double speed, VelocityControlType controlType, int slot) {
		switch (controlType) {
			case VelocityVoltage:
				motor.setControl(velocityVoltage.withVelocity(speed).withSlot(slot));
				break;
			case VelocityTorqueCurrentFOC:
				motor.setControl(velocityTorque.withVelocity(speed).withSlot(slot));
				break;
		}
	}

	public void setFollower(int leaderID, MotorAlignmentValue alignment) {
		motor.setControl(new Follower(leaderID, alignment));
	}

	public double getSupplyCurrent(){
		return motor.getSupplyCurrent().getValueAsDouble();
	}
}
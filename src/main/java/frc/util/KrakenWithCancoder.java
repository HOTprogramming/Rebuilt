package frc.util;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import frc.robot.Constants;
import frc.util.sim.PhysicsSimWithCan;

public class KrakenWithCancoder extends Kraken {

	public final CANcoder canCoder;

	@Override
	public void setupSim() {
		PhysicsSimWithCan.getInstance().addTalonFX(motor, canCoder, 1, 0.001);
	}

	@Override
	public void refreshSim() {
		PhysicsSimWithCan.getInstance().run();
	}

	public KrakenWithCancoder(int motorID, int canCoderID, TalonFXConfiguration motorConfig, CANcoderConfiguration encoderConfig, String subsystem, String deviceName) {

		this(motorID, canCoderID, Constants.canName, motorConfig, encoderConfig, subsystem, deviceName);
	}

	public KrakenWithCancoder(int motorID, int canCoderID, String canBusName, TalonFXConfiguration motorConfig,
		CANcoderConfiguration encoderConfig, String subsystem, String deviceName) {

		super(motorID, canBusName, motorConfig, subsystem, deviceName);
		this.canCoder = new CANcoder(canCoderID, new CANBus(canBusName));

		StatusCode statusCode = canCoder.getConfigurator().apply(encoderConfig);

		if (!statusCode.isOK()) {
			System.out.println("Configuration Failed on CANCoder " + subsystem + " " + deviceName);
		}
	}
}

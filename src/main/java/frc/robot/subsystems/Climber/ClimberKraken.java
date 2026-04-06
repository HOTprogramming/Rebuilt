package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.util.Kraken;
import frc.util.KrakenWithCancoder;

public class ClimberKraken {
	private static ClimberKraken instance = null;
	public DigitalInput beambreak;


	public Kraken m_climberKraken = new Kraken(ClimberConstants.kClimbermotorID, ClimberConstants.ClimberConfigs
		.elbowConfigs(), "Climber", "ClimberMotor");
	public Kraken m_spinnerKraken = new Kraken(ClimberConstants.kClimberspinnerID, ClimberConstants.ClimberConfigs
		.wristConfigs(), "Climber", "SpinnerMotor");

	public ClimberKraken() {
		m_climberKraken.setup();
		m_spinnerKraken.setup();
		beambreak = new DigitalInput(0);
	}

	public static ClimberKraken getInstance() {
		return instance = instance == null ? new ClimberKraken() : instance;
	}

	public void updateStats() {
		m_climberKraken.update();
		m_spinnerKraken.update();
	}

	public boolean getBeambreakStatus() {
		return beambreak.get();
	}
	public double getWristPos() {
		return m_spinnerKraken.motor.getPosition().getValueAsDouble();
	}
	public double getElbowPos() {
		return m_climberKraken.motor.getPosition().getValueAsDouble();
	}

	public void runManual(double percentOutputElbow, double percentOutputWrist) {
		m_climberKraken.motor.setControl(new DutyCycleOut(percentOutputElbow * 0.7));
		m_spinnerKraken.motor.setControl(new DutyCycleOut(percentOutputWrist * 0.7));
	}

	public void setElbowPosition(double desiredRotations) {
		double kG_max = 40.0; // The Amps needed to hold the robot horizontal
		double kG_min = 20.0; // The Amps needed to hold the robot vertical. This is an estimate, and may need to be tuned.
		double currentPos = this.getElbowPos() ;
		// interpolate between 0 and 1 based on where the arm is between vertical and horizontal.
		double t = (currentPos - ClimberConstants.deployedPos) / (ClimberConstants.adjustedElbowClimbPos - ClimberConstants.deployedPos);
    	t = MathUtil.clamp(t, 0.0, 1.0);

		/// Interpolate the Amps: 0.0 -> 40A, 1.0 -> 20A
    	double elbowFF = MathUtil.interpolate(kG_max, kG_min, t);

		// The elbow needs no FF when the wrist has tipped the robot 90 deg, then more FF as the wrist tips back the other way. 
		// Cosine is used because it starts at 1 when the wrist is vertical, and goes to 0 when the wrist is horizontal
		// then goes to -1 when the wrist is vertical the other way. 
		double wristScalar = Math.cos((this.getWristPos() / ClimberConstants.wristGearRatio) * 2 * Math.PI);
	
		double dynamicFF = wristScalar * elbowFF;
		SmartDashboard.putNumber("Climber/ElbowFF", dynamicFF);

		var request = new MotionMagicTorqueCurrentFOC(desiredRotations)
                      .withFeedForward(dynamicFF)
                      .withSlot(1); 
		m_climberKraken.motor.setControl(request);
	
	}

	public void setElbowVelocity(double desiredRotations) {
		m_climberKraken.setPosition(desiredRotations, Kraken.PositionControlType.MotionMagicVoltage, 1);
	}

	public void setWristPosition(double desiredRotations) {
		m_spinnerKraken.setPosition(desiredRotations, Kraken.PositionControlType.MotionMagicVoltage, 0);
	}

	public void setWristVelocity(double desiredVelocity) {
		m_spinnerKraken.setVelocity(desiredVelocity, Kraken.VelocityControlType.VelocityVoltage, 1);
	}
	public void resetWristEncoder() {
		m_spinnerKraken.motor.setPosition(0);
	}

	public void stop(){
		m_climberKraken.stop();
		m_spinnerKraken.stop();
	}
}
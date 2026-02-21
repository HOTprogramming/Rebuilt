package frc.robot.subsystems.Climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
	public final ClimberKraken m_ClimbKraken = ClimberKraken.getInstance();

	private BooleanPublisher climberBeambreak;
	double zeroPosA = 0;
	double zeroPos = 0;
	DoubleSupplier zeropossup = () -> zeroPos;
	
	public Climber() {

		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		NetworkTable table = inst.getTable("HOT/Climber");

		climberBeambreak = table.getBooleanTopic("Beambreak").publish();
	}

	@Override
	public void periodic() {
		m_ClimbKraken.updateStats();
		climberBeambreak.set(m_ClimbKraken.getBeambreakStatus());
	}

	private void runPosition(double targetPos) {
		m_ClimbKraken.runPosition(targetPos);
	}

	private void spinrunPosition(double targetPos) {
		m_ClimbKraken.spinrunPosition(targetPos);
	}

	private void spinRunVelocity(double targetVel) {
		m_ClimbKraken.spinRunVelocity(targetVel);
	}

	private BooleanSupplier isBeamBreakTripped() {
		return () -> m_ClimbKraken.getBeambreakStatus();
	}
	private BooleanSupplier isZeroed() {
		return () -> Math.abs(zeroPos - m_ClimbKraken.getSpinPosition()) < 1;
	}

	public Command zeroSpinner() {
		return Commands.sequence(spintoBeambreak(), 
		runOnce(() -> zeroPosA = m_ClimbKraken.getSpinPosition()),
		spinOutBeambreak(),
		runOnce(() -> zeroPos = (zeroPosA + m_ClimbKraken.getSpinPosition()) / 2)
		);
	}

	public Command spinToZero() {
		return run(() -> spinrunPosition(zeropossup.getAsDouble())).until(isZeroed()); 
	}

	public Command spintoBeambreak() {
		return run(() -> spinRunVelocity(40)).until(isBeamBreakTripped()); 
	}
	public Command spinOutBeambreak() {
		return run(() -> spinRunVelocity(20)).onlyWhile(isBeamBreakTripped());
	}
	
	
}


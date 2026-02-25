package frc.robot.subsystems.Climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
	public final ClimberKraken m_ClimbKraken = ClimberKraken.getInstance();

	private BooleanPublisher beambreakPub;
	private DoublePublisher zeroPosPub;

	double zeroPosA = 0;
	double zeroPos = 0;
	DoubleSupplier zeropossup = () -> zeroPos;
	
	public Climber() {

		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		NetworkTable table = inst.getTable("HOT/Climber");

		beambreakPub = table.getBooleanTopic("Beambreak").publish();
		zeroPosPub = table.getDoubleTopic("Zero Position").publish();
	}

	@Override
	public void periodic() {
		m_ClimbKraken.updateStats();
		beambreakPub.set(m_ClimbKraken.getBeambreakStatus());
		zeroPosPub.set(zeroPos);
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
	private BooleanSupplier isStowed() {
		return () -> Math.abs(m_ClimbKraken.getClimbPosition() - ClimberConstants.storedPos) < 0.002;
	}
	private BooleanSupplier isDeployed() {
		return () -> Math.abs(m_ClimbKraken.getClimbPosition() - ClimberConstants.deployedPos) < 0.002;
	}
	private BooleanSupplier isClimbed() {
		return () -> Math.abs(m_ClimbKraken.getClimbPosition() - ClimberConstants.climbedPos) < 0.01;
	}

	public Command zeroSpinner() {
		return Commands.sequence(spintoBeambreak(), spinOutBeambreak(), 
		spinOutSlow(), 
		runOnce(() -> zeroPosA = m_ClimbKraken.getSpinPosition()),
		spinBackSlow(),
		runOnce(() -> zeroPos = (((zeroPosA + m_ClimbKraken.getSpinPosition()) / 2) + ClimberConstants.wristZeroOffset)),
		spinToZero()
		);
	}

	public Command fullDeploy() {
		return Commands.sequence(
			runOnce(() -> runPosition(ClimberConstants.deployedPos)),
			zeroSpinner(), 
			deploy());
	}

	public Command spinOutSlow() {
		return Commands.sequence(
			run(() -> spinRunVelocity(-20)).withDeadline(Commands.waitSeconds(0.5)), 
			run(() -> spinRunVelocity(-20)).onlyWhile(isBeamBreakTripped()));
	}
	
	public Command spinBackSlow() {
		return Commands.sequence(
			run(() -> spinRunVelocity(20)).withDeadline(Commands.waitSeconds(0.5)), 
			run(() -> spinRunVelocity(20)).onlyWhile(isBeamBreakTripped()));
	}
	public Command spinToZero() {
		return run(() -> {
				spinrunPosition(zeropossup.getAsDouble());
			}).until(isZeroed()); 
	}
	public Command teleopDeploy() {
		return run(() -> {
			runPosition(ClimberConstants.deployedPos);
			spinrunPosition(zeropossup.getAsDouble());
		});
	}
	public Command spintoBeambreak() {
		return run(() -> {
			spinRunVelocity(80);
		}).until(isBeamBreakTripped()); 
	}
	public Command spinOutBeambreak() {
		return run(() -> spinRunVelocity(80)).onlyWhile(isBeamBreakTripped());
	}
	public Command stow() {
		return run(() -> runPosition(ClimberConstants.storedPos)).until(isStowed());
	}
	public Command deploy() {
		return run(() -> runPosition(ClimberConstants.deployedPos)).until(isDeployed());
	}
	public Command climb() {
		return run(() -> runPosition(ClimberConstants.climbedPos)).until(isClimbed());
	}
}
package frc.robot.subsystems.Climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
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
	private BooleanSupplier isWristClimbed() {
		return () -> Math.abs(((m_ClimbKraken.getSpinPosition() - zeroPos) / ClimberConstants.wristGearRatio) - ClimberConstants.wristClimbedPos) < 0.01;
	}
	private BooleanSupplier isLifted() {
		return () -> Math.abs(m_ClimbKraken.getClimbPosition() - ClimberConstants.liftClimbPos) < 0.01;
	}


	private void spinWristPosition(double wristPosRevs) {
		spinrunPosition(zeroPos + (wristPosRevs * ClimberConstants.wristGearRatio));

	}

	public Command stageThreeClimb() {
		return Commands.sequence(
		lift(),
		wristClimb(),
		deploy()
		);
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

	private Command spinOutSlow() {
		return Commands.sequence(
			run(() -> spinRunVelocity(-20)).withDeadline(Commands.waitSeconds(0.5)), 
			run(() -> spinRunVelocity(-20)).onlyWhile(isBeamBreakTripped()));
	}
	
	private Command spinBackSlow() {
		return Commands.sequence(
			run(() -> spinRunVelocity(20)).withDeadline(Commands.waitSeconds(0.5)), 
			run(() -> spinRunVelocity(20)).onlyWhile(isBeamBreakTripped()));
	}

	public Command wristClimb() {
		return run(() -> {
				spinWristPosition(ClimberConstants.wristClimbedPos);
			}).until(isWristClimbed()); 
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

	private Command spintoBeambreak() {
		return run(() -> {
			spinRunVelocity(80);
		}).until(isBeamBreakTripped()); 
	}

	private Command spinOutBeambreak() {
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

	public Command lift() {
		return run(() -> runPosition(ClimberConstants.liftClimbPos)).until(isLifted());
	}
}
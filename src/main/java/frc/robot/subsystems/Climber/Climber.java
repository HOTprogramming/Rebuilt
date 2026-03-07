package frc.robot.subsystems.Climber;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
		SmartDashboard.putBoolean("elbowclimber",Math.abs(m_ClimbKraken.getClimbPosition() - ClimberConstants.l3ElbowPos) < 0.002);
	}

	@Override

	//voids
	public void periodic() {
		m_ClimbKraken.updateStats();
		beambreakPub.set(m_ClimbKraken.getBeambreakStatus());
		zeroPosPub.set(zeroPos);
	}

	private void setElbowPosition(double targetPos) {
		m_ClimbKraken.runPosition(targetPos);
	}

	private void setWristPosition(double targetPos) {
		m_ClimbKraken.spinrunPosition(targetPos);
	}

	private void setWristVelocity(double targetVel) {
		m_ClimbKraken.spinRunVelocity(targetVel);
	}

	private void spinWristPosition(double wristPosRevs) {
		setWristPosition(zeroPos + (wristPosRevs * ClimberConstants.wristGearRatio));
	}



	//booleans
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
		return () -> Math.abs(m_ClimbKraken.getSpinPosition() - ClimberConstants.wristClimbPos) < 0.5;
	}

	private BooleanSupplier isLifted() {
		return () -> Math.abs(m_ClimbKraken.getClimbPosition() - ClimberConstants.liftElbowClimbPos) < 0.01;
	}
	
	public BooleanSupplier isElbowl3Climbed() {
		return () -> Math.abs(m_ClimbKraken.getClimbPosition() - ClimberConstants.l3ElbowPos) < 0.002;
	}

	public BooleanSupplier isPartClimbed() {
		return () -> Math.abs(m_ClimbKraken.getClimbPosition() - ClimberConstants.partElbowLiftPos) < 0.01;
	}

	//main commands
	public Command stageThreeClimb() {
		return Commands.sequence(
		lift(),
		wristClimb(),
		finalElbowExtend()
		);
	}

	public Command Zero() {
		return runOnce(() -> {m_ClimbKraken.resetSpinnerEncoder();
		zeroPos = 0;});
		}
	public Command spinToZero() {
		return run(() -> {
				setWristPosition(zeropossup.getAsDouble());
			}).until(isZeroed()); 
	}

	public Command fullDeploy() {
		return Commands.sequence(
			runOnce(() -> setElbowPosition(ClimberConstants.deployedPos)),
			spinToZero(), 
			deploy());
	}
	
	public Command teleopDeploy() {
		return run(() -> {
			setElbowPosition(ClimberConstants.deployedPos);
			setWristPosition(zeropossup.getAsDouble());
		});
	}

	public Command stop() {
		return runOnce(() -> {});
	}
	
	public Command stow() {
		return run(() -> setElbowPosition(ClimberConstants.storedPos)).until(isStowed());
	}

	public Command autonClimb() {
		return run(() -> setElbowPosition(ClimberConstants.liftElbowClimbPos)).until(isLifted());
	}

	//sub commands

	private Command spinOutSlow() {
		return Commands.sequence(
			run(() -> setWristVelocity(-20)).withDeadline(Commands.waitSeconds(0.5)), 
			run(() -> setWristVelocity(-20)).onlyWhile(isBeamBreakTripped()));
	}
	
	private Command spinBackSlow() {
		return Commands.sequence(
			run(() -> setWristVelocity(20)).withDeadline(Commands.waitSeconds(0.5)), 
			run(() -> setWristVelocity(20)).onlyWhile(isBeamBreakTripped()));
	}


	private Command wristClimb() {
		return run(() -> {
			spinWristPosition(ClimberConstants.wristClimbPos);
	}).until(isWristClimbed());
	}


	private Command finalElbowExtend() {
		return run(() -> {
				setElbowPosition(ClimberConstants.l3ElbowPos);
				System.out.println("running");
			}).until(isElbowl3Climbed()); 
	}

	private Command spintoBeambreak() {
		return run(() -> {
			setWristVelocity(100);
		}).until(isBeamBreakTripped()); 
	}

	private Command startClimb() {
		return run(() -> setElbowPosition(ClimberConstants.partElbowLiftPos)).until(isPartClimbed());
	}

	private Command spinOutBeambreak() {
		return run(() -> setWristVelocity(100)).onlyWhile(isBeamBreakTripped());
	}

	private Command deploy() {
		return run(() -> setElbowPosition(ClimberConstants.deployedPos)).until(isDeployed());
	}

	private Command climb() {
		return run(() -> setElbowPosition(ClimberConstants.climbedPos)).until(isClimbed());
	}

	private Command lift() {
		return run(() -> setElbowPosition(ClimberConstants.liftElbowClimbPos)).until(isLifted());
	}
	public Command reset() {
		return run(() -> setWristPosition(0));
	}
}
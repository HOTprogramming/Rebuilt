package frc.robot.subsystems.Climber;

import java.util.function.BooleanSupplier;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
	public final ClimberKraken m_ClimbKraken = ClimberKraken.getInstance();

	private double finLeftEdgePos;
	private double finRightEdgePos;

	private BooleanPublisher beambreakPub;
	
	public Climber() {
		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		NetworkTable table = inst.getTable("HOT/Climber");

		beambreakPub = table.getBooleanTopic("Beambreak").publish();
		//SmartDashboard.putBoolean("elbowclimber", Math.abs(m_ClimbKraken.getElbowPos() - ClimberConstants.l3ElbowPos) < 0.002);
		
		m_ClimbKraken.m_climberKraken.motor.setPosition(0);
	}

	@Override
	public void periodic() {
		m_ClimbKraken.updateStats();
		beambreakPub.set(m_ClimbKraken.getBeambreakStatus());
		
		// if(this.getCurrentCommand() != null){
		// 	SmartDashboard.putString("climber/current command", this.getCurrentCommand().getName());
		// }
	}

	private void setElbowPosition(double targetPos) {
		m_ClimbKraken.setElbowPosition(targetPos);
	}

	private void setUnloadedElbowPosition(double targetPos) {
		m_ClimbKraken.setUnloadedElbowPosition(targetPos);
	}
	
	private void setElbowVelocity(double targetVel) {
		m_ClimbKraken.setElbowVelocity(targetVel);
	}

	private void setWristPosition(double targetPos) {
		m_ClimbKraken.setWristPosition(targetPos);
	}

	private void setWristVelocity(double targetVel) {
		m_ClimbKraken.setWristVelocity(targetVel);
	}

	//booleans
	private BooleanSupplier isBeamBreakTripped() {
		return () -> m_ClimbKraken.getBeambreakStatus();
	}
	
	private BooleanSupplier isZeroed() {
		return () -> Math.abs(m_ClimbKraken.getWristPos()) < 0.005;
	}

	private BooleanSupplier isStowed() {
		return () -> Math.abs(m_ClimbKraken.getElbowPos()) < 0.002;
	}

	private BooleanSupplier isDeployed() {
		//System.out.println("is deployed");
		return () -> Math.abs(m_ClimbKraken.getElbowPos() - ClimberConstants.deployedPos) < 0.01;
	}

	private BooleanSupplier isLifted() {
		return () -> Math.abs(m_ClimbKraken.getElbowPos() - ClimberConstants.initalElbowClimbPos) < 0.01;
	}

	private BooleanSupplier startElbowAdjustment() {
		return () -> (m_ClimbKraken.getWristPos() / ClimberConstants.wristGearRatio) > ClimberConstants.startElbowAdjustmentPos;
	}
	
	private BooleanSupplier isWristClimbed() {
		return () -> Math.abs((m_ClimbKraken.getWristPos() / ClimberConstants.wristGearRatio) - ClimberConstants.wristClimbPos) < 0.01;
	}

	public BooleanSupplier isElbowl3Climbed() {
		return () -> Math.abs(m_ClimbKraken.getElbowPos() - ClimberConstants.l3ElbowPos) < 0.01;
	}

	public BooleanSupplier isAutoClimbed() {
		return () -> Math.abs(m_ClimbKraken.getElbowPos() - ClimberConstants.autoClimbPos) < 0.01;
	}



	
	private BooleanSupplier isPartWristClimbed() {
		return () -> Math.abs((m_ClimbKraken.getWristPos() / ClimberConstants.wristGearRatio) - ClimberConstants.startElbowAdjustmentPos) < 0.05;
	}

	public BooleanSupplier isPartClimbed() {
		return () -> Math.abs(m_ClimbKraken.getElbowPos() - ClimberConstants.startWristAtPos) < 0.01;
	}

	public BooleanSupplier areWheelsUp() {
		return () -> m_ClimbKraken.getElbowPos() < ClimberConstants.wheelsUpPos;
	}

	public Command zeroSpinner() {
		return Commands.sequence(
			spintoBeambreak(),
			spinOutBeambreak(),
			spinOutSlow(), runOnce(() -> finLeftEdgePos = m_ClimbKraken.getWristPos()),
			spinBackSlow(), runOnce(() -> finRightEdgePos = m_ClimbKraken.getWristPos()),
			runOnce(() -> m_ClimbKraken.setWristPosition(((finLeftEdgePos + finRightEdgePos) / 2) + ClimberConstants.wristZeroOffset)),
			Commands.waitSeconds(1.0),
			reset()
		);
	}

	public Command spinToZero() {
		return run(() -> {setWristPosition(0.0);}).until(isZeroed()); 
	}
	
	private Command spintoBeambreak() {
		return run(() -> {setWristVelocity(125);}).until(isBeamBreakTripped()).withTimeout(7.0); 
	}
	//Run for short time to get past the far edge of thebeambreak
	private Command spinOutBeambreak() {
		return run(() -> setWristVelocity(40)).withTimeout(0.5);
	}
	//Rotate slowly back toward beambreak until it trips to get first edge or times out
	private Command spinOutSlow() {
		return Commands.run(() -> setWristVelocity(-20)).until(isBeamBreakTripped()).withTimeout(2.0);
	}
	//Continue rotating slowly in the same direction until it trips again to get second edge or times out
	private Command spinBackSlow() {
		return Commands.run(() -> setWristVelocity(-20)).onlyWhile(isBeamBreakTripped()).withTimeout(1.0);
	}

	public Command fullDeploy() {
		return Commands.sequence(
			runOnce(() -> setUnloadedElbowPosition(ClimberConstants.deployedPos)),
			zeroSpinner(), 
			deploy());
	}	
	public Command deploy() {
		return run(() -> setUnloadedElbowPosition(ClimberConstants.deployedPos)).until(isDeployed());
	}

	public Command autonDeploy() {
		return runOnce(() -> setUnloadedElbowPosition(ClimberConstants.deployedPos));
	}	

	public Command stop() {
		return runOnce(() -> {});
	}
	
	public Command stow() {
		return run(() -> setUnloadedElbowPosition(ClimberConstants.storedPos)).until(isStowed());
	}

	public Command autoClimb() {
		return run(() -> setElbowPosition(ClimberConstants.autoClimbPos));
	}

	public Command stageThreeClimb() {
		return Commands.sequence(
		lift(),
		wristClimb(),
		finalElbowExtend()
		);
	}

	public Command lift() {
		return run(() -> setElbowPosition(ClimberConstants.initalElbowClimbPos)).until(isLifted());
	}

	public Command wristClimb() {
		return Commands.sequence(
			initialWristClimb(), 
			elbowAdjustment()
		);
	}

	private Command initialWristClimb() {
		return run(() -> {
			setWristPosition(ClimberConstants.wristClimbPos * ClimberConstants.wristGearRatio);
			setElbowPosition(ClimberConstants.initalElbowClimbPos);
		}).until(startElbowAdjustment());
	}

	private Command elbowAdjustment() {
		return run(() -> {
			setWristPosition(ClimberConstants.wristClimbPos * ClimberConstants.wristGearRatio);
			setElbowPosition(ClimberConstants.adjustedElbowClimbPos);
		}).until(isWristClimbed());	
	}

	public Command finalElbowExtend() {
		return run(() -> {
			setWristPosition(ClimberConstants.wristClimbPos * ClimberConstants.wristGearRatio);
			setElbowPosition(ClimberConstants.l3ElbowPos);
		});
	}

	public Command liftUpWheelsSlow() {
		return run(() -> setElbowVelocity(ClimberConstants.liftSpeed)).until(areWheelsUp());
	}

	public Command partWristClimb() {
		return run(() -> setWristPosition(ClimberConstants.startElbowAdjustmentPos)).until(isPartWristClimbed());
	}

	private Command reset() {
		return runOnce(() -> {
			m_ClimbKraken.resetWristEncoder();
			m_ClimbKraken.setWristPosition(0);
		});
	}
}
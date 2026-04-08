package frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase {
	public final IntakeKraken mIntakeKraken = IntakeKraken.getInstance();

	public boolean shouldStop = false;

	public Intake() {

	}

	@Override
	public void periodic() {
		mIntakeKraken.updateStats();
	}

	public Command extend() {
		return Commands.sequence(runOnce(() -> {
			mIntakeKraken.m_ExtendKraken.motor.getConfigurator()
				.apply(new MotionMagicConfigs()
					.withMotionMagicAcceleration(64)
					.withMotionMagicCruiseVelocity(64));
		}), new WaitCommand(.02), runOnce(() -> {
			mIntakeKraken.runAgitatePosition(IntakeConstants.kDeployedPosition);
			mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
		}));
	}

	public Command extendShort() {
		return Commands.sequence(runOnce(() -> {
			mIntakeKraken.m_ExtendKraken.motor.getConfigurator()
				.apply(new MotionMagicConfigs()
					.withMotionMagicAcceleration(64)
					.withMotionMagicCruiseVelocity(64));
		}), new WaitCommand(.02), runOnce(() -> {
			mIntakeKraken.runAgitatePosition(IntakeConstants.kShortDeployedPosition);
			mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
		}));
	}

	public Command agitateRetract() {
		return Commands.sequence(runOnce(() -> {
			mIntakeKraken.m_ExtendKraken.motor.getConfigurator()
				.apply(new MotionMagicConfigs()
					.withMotionMagicAcceleration(12)
					.withMotionMagicCruiseVelocity(80));
		}), new WaitCommand(.02), runOnce(() -> {
			mIntakeKraken.runAgitatePosition(IntakeConstants.kRetractPositionAgitate);
			mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
		}));
	}

	public Command retractSlow() {
		return Commands.sequence(
			Commands.waitSeconds(0.0),
			runOnce(() -> {
			mIntakeKraken.m_ExtendKraken.motor.getConfigurator()
				.apply(new MotionMagicConfigs()
					.withMotionMagicAcceleration(1.5)
					.withMotionMagicCruiseVelocity(2));
		}), new WaitCommand(.02), runOnce(() -> {
			mIntakeKraken.runAgitatePosition(IntakeConstants.kRetractPositionAgitate);
			mIntakeKraken.stopRollers();
		}));
	}


	public Command teleopRetract() {
		return runOnce(() -> {
			mIntakeKraken.runPosition(IntakeConstants.kRetractPositionAgitate);
			mIntakeKraken.stopRollers();
		});
	}

	public Command fullRetract() {
		return runOnce(() -> {
			mIntakeKraken.runPosition(IntakeConstants.kRetractPosition);
			mIntakeKraken.stopRollers();
		});
	}

	public Command rollerStop() {
		return run(() -> {
			mIntakeKraken.stopRollers();
		});
	}

	public Command rollerSpin() {
		return runOnce(() -> {
			mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
		});
	}

	public Command stop() {
		return runOnce(() -> {
			mIntakeKraken.stopRollers();
		});
	}

	public Command stopButSpin() {
		return runOnce(() -> {
			mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
			mIntakeKraken.m_ExtendKraken.motor.setVoltage(0);
		});
	}

	public void completeStopMethod(){
		// mIntakeKraken.m_ExtendKraken.motor.setVoltage(0);
		mIntakeKraken.m_RollerKrakenFollower.motor.setVoltage(0);
		mIntakeKraken.m_RollerKrakenLeader.motor.setVoltage(0);
	}

	public Command completeStop() {
		return runOnce(() -> {
			completeStopMethod();
		});
	}

	public BooleanSupplier isExtended() {
		return () -> Math.abs(mIntakeKraken.getExtendPosition()
			- IntakeConstants.kDeployedPosition) < IntakeConstants.kExtendTolerance;
	}

	public BooleanSupplier isRetracted() {
		return () -> Math.abs(mIntakeKraken.getExtendPosition()
			- IntakeConstants.kRetractPosition) < IntakeConstants.kRetractTolerance;
	}

	public BooleanSupplier isAgitateRetracted() {
		return () -> Math.abs(mIntakeKraken.getExtendPosition()
			- IntakeConstants.kRetractPositionAgitate) < IntakeConstants.kRetractTolerance;
	}

	public Boolean IsIntakeExtended() {
		return Math.abs(mIntakeKraken.getExtendPosition()
			- IntakeConstants.kDeployedPosition) < IntakeConstants.kExtendTolerance;
	}
	

	public Command agitate(){
		return Commands.sequence(
			DriverStation.isAutonomous() ? new WaitCommand(0) : new WaitCommand(.5),
			agitateRetract(),
			new WaitCommand(.5),
			extend(),
			new WaitCommand(.5),
			agitateRetract()
		);
	}

	public Command packingAgitate() {
		return Commands.sequence(Commands.repeatingSequence(
			agitateRetract(),
			run(() -> {}).until(currentTrippedDebounced()),
			reExtend(),
			run(() -> {}).until(isReExetended())
		).until(doneAgitating()),
		runOnce(() -> SmartDashboard.putBoolean("done agitating", true)));

	}

	private Trigger currentTrippedDebounced() {
		return new Trigger(
			() -> mIntakeKraken.m_ExtendKraken.motor.getStatorCurrent().getValueAsDouble() > IntakeConstants.agitateCurrentThreshold
		).debounce(IntakeConstants.agitateCurrentTime);
	}

	private Trigger doneAgitating() {
		return new Trigger(isAgitateRetracted()).and(() -> mIntakeKraken.m_ExtendKraken.motor.getStatorCurrent().getValueAsDouble() < IntakeConstants.agitateCurrentThreshold)
		.and(() -> Math.abs(mIntakeKraken.m_ExtendKraken.motor.getClosedLoopReference().getValueAsDouble() - IntakeConstants.kRetractPositionAgitate) < 0.01);
	}
	
	double targetreextendpos = 0.9;
	double reextendoffset = 0.4;

	private Command reExtend() {
		return 
		runOnce(() -> {
			targetreextendpos = MathUtil.clamp(mIntakeKraken.getExtendPosition() + reextendoffset, 0.1, IntakeConstants.kDeployedPosition);
			mIntakeKraken.runAgitatePosition(targetreextendpos);
		});
	}

	private BooleanSupplier isReExetended() {
		return () -> Math.abs(mIntakeKraken.getExtendPosition() - targetreextendpos) < IntakeConstants.kExtendTolerance;
	}
	
}

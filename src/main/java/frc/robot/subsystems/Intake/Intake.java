package frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

<<<<<<< HEAD
=======
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
>>>>>>> a721243b7ca975bde9cfcef03f69f24138f110a9
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Intake extends SubsystemBase {
	public final IntakeKraken mIntakeKraken;

	public boolean shouldStop = false;

	public Intake() {
		mIntakeKraken = IntakeKraken.getInstance();
	}

	@Override
	public void periodic() {
		mIntakeKraken.updateStats();
	}

	public Command extend() {
		return Commands.sequence(
			runOnce(() -> {
				mIntakeKraken.m_ExtendKraken.motor.getConfigurator().apply(new MotionMagicConfigs()
					.withMotionMagicAcceleration(64)
					.withMotionMagicCruiseVelocity(64));
			}),
			new WaitCommand(.02),
			runOnce(() -> {
				mIntakeKraken.runAgitatePosition(IntakeConstants.kDeployedPosition);
				mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
				mIntakeKraken.runKickerVelocity(IntakeConstants.kKickerSpeed);
			}));
	}

	public Command extendShort() {
		return Commands.sequence(
			runOnce(() -> {
				mIntakeKraken.m_ExtendKraken.motor.getConfigurator().apply(new MotionMagicConfigs()
					.withMotionMagicAcceleration(64)
					.withMotionMagicCruiseVelocity(64));
			}),
			new WaitCommand(.02),
			runOnce(() -> {
				mIntakeKraken.runAgitatePosition(IntakeConstants.kShortDeployedPosition);
				mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
				mIntakeKraken.runKickerVelocity(IntakeConstants.kKickerSpeed);
			}));
	}

	public Command agitateExtend(){
		return runOnce(() -> {
			mIntakeKraken.m_ExtendKraken.motor.getConfigurator().apply(new MotionMagicConfigs()
				.withMotionMagicAcceleration(64)
				.withMotionMagicCruiseVelocity(64));
			mIntakeKraken.runAgitatePosition(IntakeConstants.kDeployedPosition);
			mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
			mIntakeKraken.runKickerVelocity(IntakeConstants.kKickerSpeed);
		});
	}

	public Command agitateRetract(){
		return Commands.sequence(
			runOnce(() -> {
				mIntakeKraken.m_ExtendKraken.motor.getConfigurator().apply(new MotionMagicConfigs()
					.withMotionMagicAcceleration(8)
					.withMotionMagicCruiseVelocity(8));
			}),
			new WaitCommand(.02),
			runOnce(() -> {
				mIntakeKraken.runAgitatePosition(IntakeConstants.kRetractPositionAgitate);
				mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed / 2.0);
				mIntakeKraken.runKickerVelocity(IntakeConstants.kKickerSpeed);
			}));
	}


	public Command teleopRetract() {
		return runOnce(() -> {
			mIntakeKraken.runPosition(IntakeConstants.kRetractPositionAgitate);
			mIntakeKraken.stop();
		});
	}
	public Command fullretract() {
		return runOnce(() -> {
			mIntakeKraken.runPosition(IntakeConstants.kRetractPosition);
		});
	}

	public Command expel() {
		return runOnce(() -> {
			mIntakeKraken.runPosition(IntakeConstants.kDeployedPosition);
			mIntakeKraken.runVelocity(IntakeConstants.kExpelSpeed);
		});
	}

	public Command rollerStop() {
	return run(() -> {
		mIntakeKraken.runPosition(IntakeConstants.kDeployedPosition);
		mIntakeKraken.stop();
		});
	}

	public Command rollerStop1() {
	return run(() -> {
		mIntakeKraken.stop();
		});
	}

	public Command rollerSpin() {
		return runOnce(() -> {
			mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
		});
	}

	public Command stop() {
		return runOnce(() -> {
			mIntakeKraken.stop();
		});
	}

	public Command stopButSpin() {
		return runOnce(() -> {
			mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
			mIntakeKraken.m_ExtendKraken.motor.setVoltage(0);
		});
	}

<<<<<<< HEAD
	public Command completeStop(){
		return runOnce(() -> {
				mIntakeKraken.m_ExtendKraken.motor.setVoltage(0);
				mIntakeKraken.m_RollerKrakenFollower.motor.setVoltage(0);
				mIntakeKraken.m_RollerKrakenLeader.motor.setVoltage(0);
				mIntakeKraken.m_KickerKraken.motor.setVoltage(0);
			}
		);
=======
	public void completeStopMethod(){
		mIntakeKraken.m_ExtendKraken.motor.setVoltage(0);
		mIntakeKraken.m_RollerKrakenFollower.motor.setVoltage(0);
		mIntakeKraken.m_RollerKrakenLeader.motor.setVoltage(0);
		mIntakeKraken.m_KickerKraken.motor.setVoltage(0);
	}

	public Command completeStop() {
		return runOnce(() -> {
			completeStopMethod();
		});
>>>>>>> a721243b7ca975bde9cfcef03f69f24138f110a9
	}

	public BooleanSupplier isExtended() {
		return () -> Math.abs(mIntakeKraken.getExtendPosition() - IntakeConstants.kDeployedPosition) < IntakeConstants.kExtendTolerance;
	}

	public BooleanSupplier isRetracted() {
		return () -> Math.abs(mIntakeKraken.getExtendPosition() - IntakeConstants.kRetractPosition) < IntakeConstants.kRetractTolerance;
	}

	public BooleanSupplier isAgitateRetracted() {
		return () -> Math.abs(mIntakeKraken.getExtendPosition() - IntakeConstants.kRetractPositionAgitate) < IntakeConstants.kRetractTolerance;
	}

	public Command agitate() {
		// return Commands.repeatingSequence(agitateExtend().until(isExtended()),agitateRetract().until(isRetracted()));
		return agitateRetract();
	}


	public Boolean IsIntakeExtended() {
		return Math.abs(mIntakeKraken.getExtendPosition() - IntakeConstants.kDeployedPosition) < IntakeConstants.kExtendTolerance;
	}

<<<<<<< HEAD
=======
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
		.and(() -> mIntakeKraken.m_ExtendKraken.motor.getClosedLoopReference().getValueAsDouble() == IntakeConstants.kRetractPositionAgitate);
	}
	
	double targetreextendpos = 0.9;
	double reextendoffset = 0.3;

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
>>>>>>> a721243b7ca975bde9cfcef03f69f24138f110a9
}


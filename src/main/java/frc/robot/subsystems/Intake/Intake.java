package frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake extends SubsystemBase {
	public final IntakeKraken mIntakeKraken;

	public Intake() {
		mIntakeKraken = IntakeKraken.getInstance();
	}

	@Override
	public void periodic() {
		mIntakeKraken.updateStats();
	}

	public Command extend() {
		return runOnce(() -> {
			mIntakeKraken.runPosition(IntakeConstants.kDeployedPosition);
			mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
		});
	}

	public Command agitateExtend(){
		return run(() -> {
			mIntakeKraken.runAgitatePosition(IntakeConstants.kDeployedPosition);
			mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
		});
	}

	public Command agitateRetract(){
		return run(() -> {
			mIntakeKraken.runAgitatePosition(IntakeConstants.kRetractPositionAgitate);
		});
	}

	public Command retract() {
		return run(() -> {
			mIntakeKraken.runPosition(IntakeConstants.kRetractPosition);
			mIntakeKraken.stop();
		});
	}

	public Command expel() {
		return run(() -> {
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
}

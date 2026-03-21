package frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class Intake extends SubsystemBase {
	public final IntakeKraken mIntakeKraken = new IntakeKraken();

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
			mIntakeKraken.runKickerVelocity(IntakeConstants.kKickerSpeed);
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
			mIntakeKraken.runKickerVelocity(IntakeConstants.kKickerSpeed);
		}));
	}

	public Command agitateRetract() {
		return Commands.sequence(runOnce(() -> {
			mIntakeKraken.m_ExtendKraken.motor.getConfigurator()
				.apply(new MotionMagicConfigs()
					.withMotionMagicAcceleration(8)
					.withMotionMagicCruiseVelocity(8));
		}), new WaitCommand(.02), runOnce(() -> {
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

	public Command fullRetract() {
		return runOnce(() -> {
			mIntakeKraken.runPosition(IntakeConstants.kRetractPosition);
			mIntakeKraken.stop();
		});
	}

	public Command rollerStop() {
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

	public Command completeStop() {
		return runOnce(() -> {
			mIntakeKraken.m_ExtendKraken.motor.setVoltage(0);
			mIntakeKraken.m_RollerKrakenFollower.motor.setVoltage(0);
			mIntakeKraken.m_RollerKrakenLeader.motor.setVoltage(0);
			mIntakeKraken.m_KickerKraken.motor.setVoltage(0);
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

}

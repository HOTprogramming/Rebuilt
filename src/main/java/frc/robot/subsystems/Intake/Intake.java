package frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
		// if(IsIntakeExtended()) {
		// 	mIntakeKraken.stopExtend();
		// }	
		// SmartDashboard.putBoolean("Is Extended", IsIntakeExtended()); 

		SmartDashboard.putBoolean("intakeExtend", IsIntakeExtended()); 
	}

	public Command extend() {
		return Commands.sequence(
			runOnce(() -> {
				mIntakeKraken.m_ExtendKraken.motor.getConfigurator().apply(new MotionMagicConfigs()
					.withMotionMagicAcceleration(64)
					.withMotionMagicCruiseVelocity(64));
				mIntakeKraken.m_ExtendKraken.motor.getConfigurator().apply(
					new CurrentLimitsConfigs()
						.withStatorCurrentLimit(40)
						.withStatorCurrentLimitEnable(true)
						.withSupplyCurrentLimit(40)
						.withSupplyCurrentLimitEnable(true)
						.withSupplyCurrentLowerLimit(30)
						.withSupplyCurrentLowerTime(1));
			}),
			new WaitCommand(.02),
			runOnce(() -> {
				mIntakeKraken.runAgitatePosition(IntakeConstants.kDeployedPosition);
				mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
			}));
	}

	public Command agitateExtend(){
		return runOnce(() -> {
			mIntakeKraken.m_ExtendKraken.motor.getConfigurator().apply(new MotionMagicConfigs()
				.withMotionMagicAcceleration(64)
				.withMotionMagicCruiseVelocity(64));
			mIntakeKraken.runAgitatePosition(IntakeConstants.kDeployedPosition);
			mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
		});
	}

	public Command agitateRetract(){
		return Commands.sequence(
			runOnce(() -> {
				mIntakeKraken.m_ExtendKraken.motor.getConfigurator().apply(new MotionMagicConfigs()
					.withMotionMagicAcceleration(1)
					.withMotionMagicCruiseVelocity(1));
				mIntakeKraken.m_ExtendKraken.motor.getConfigurator().apply(
					new CurrentLimitsConfigs()
						.withStatorCurrentLimit(8)
						.withStatorCurrentLimitEnable(true)
						.withSupplyCurrentLimit(8)
						.withSupplyCurrentLimitEnable(true)
						.withSupplyCurrentLowerLimit(5)
						.withSupplyCurrentLowerTime(1));
			}),
			new WaitCommand(.02),
			runOnce(() -> {
				mIntakeKraken.runAgitatePosition(IntakeConstants.kRetractPositionAgitate);
				mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed / 2.0);
				shouldStop = false;
			}));
	}

	public Command retract() {
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

	public Command stopButSpin(){
		return runOnce(() -> {
			mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed);
			mIntakeKraken.m_ExtendKraken.motor.setVoltage(0);
		});
	}

	public Command completeStop(){
		return runOnce(() -> {
				mIntakeKraken.m_ExtendKraken.motor.setVoltage(0);
				mIntakeKraken.m_RollerKrakenFollower.motor.setVoltage(0);
				mIntakeKraken.m_RollerKrakenLeader.motor.setVoltage(0);
			}
		);
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

}


package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
	public final ClimberKraken m_ClimbKraken = ClimberKraken.getInstance();

	public Climber() {}

	@Override
	public void periodic() {
		m_ClimbKraken.updateStats();
	}

	private void runPosition(double targetPos) {
		m_ClimbKraken.runPosition(targetPos);
	}

	private void spinrunPosition(double targetPos) {
		m_ClimbKraken.spinrunPosition(targetPos);
	}

	public Command deployCommand() {
		return runOnce(() -> runPosition(ClimberConstants.deployPosition));
	}

	public Command retractCommand() {
		return runOnce(() -> runPosition(ClimberConstants.retractPosition));
	}

	public Command rotateCommand() {
		return runOnce(() -> spinrunPosition(ClimberConstants.spinPosition));
	}

	public Command spinhomeCommand() {
		return runOnce(() -> spinrunPosition(ClimberConstants.spinhomePosition));
	}

	public Command latchCommand() {
		return runOnce(() -> spinrunPosition(ClimberConstants.lockPosition));
	}

	public Command LV1Command() {
		return runOnce(() -> spinrunPosition(ClimberConstants.lvlonePosition));
	}
}
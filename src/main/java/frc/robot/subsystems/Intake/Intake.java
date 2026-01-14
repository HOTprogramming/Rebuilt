package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{

    private Intake() {

    }

    private void setLeftRollerSpeed() {

    }

    private void setLeftIntakeAngle(double intakeAngle) {

    }
    public Command extendLeftIntake() {
        return runOnce(() ->
        {
            setLeftIntakeAngle(IntakeConstants.kLeftdeployedAngle);
        });
    }

    public Command unextendLeftIntake() {
        return runOnce(() ->
        {
            setLeftIntakeAngle(IntakeConstants.kLeftdeployedAngle);
        });
    }


    public Command turnOnLeftIntake() {
        return runOnce(() ->
        {
            setLeftRollerSpeed();
        });
    }

  public Command turnOffLeftIntake() {
        return runOnce(() ->
        {
            setLeftRollerSpeed();
        });
    }
    // add a command for every function the intake will need to do 
}
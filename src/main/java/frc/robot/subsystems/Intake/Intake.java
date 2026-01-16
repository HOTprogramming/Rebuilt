package frc.robot.subsystems.Intake;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final IntakeKraken mIntakeKraken = IntakeKraken.getInstance();

    private Intake() {

    }

    private void setLeftRollerSpeed() {

    }

    private void setLeftIntakePosition(double intakePosition) {
        mIntakeKraken.runPosition(intakePosition,1);
        mIntakeKraken.runVelocity(intakePosition,1);
    }
    public Command extendLeftIntake() {
        return runOnce(() ->
        {
            setLeftIntakePosition(IntakeConstants.kLeftdeployedPosition);
        });
    }
     public Command retractLeftIntake() {
        return runOnce(() ->
        {
            setLeftIntakePosition(IntakeConstants.kLeftdeployedPosition);
        });
    }
    public Command expelLeftIntake() {
        return runOnce(() ->
          {
            setLeftIntakePosition(IntakeConstants.kLeftdeployedExpel);
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

    private void setRightIntakePosition(double intakePosition) {
           mIntakeKraken.runPosition(intakePosition,0);
           mIntakeKraken.runVelocity(intakePosition,0);
    }

    public Command extendRightIntake() {
        return runOnce(() ->
        {
            setRightIntakePosition(IntakeConstants.kRightdeployedPosition);
        });
    }

    public Command retractRightIntake() {
        return runOnce(() ->
        {
            setRightIntakePosition(IntakeConstants.kRightdeployedPosition);
        }); 
    }
    public Command expelRightIntake() {
        return runOnce(() ->
          {
            setRightIntakePosition(IntakeConstants.kRightdeployedExpel);
        });
    }

    public Command turnOnRightIntake() {
        return runOnce(() ->
        {
            setLeftRollerSpeed();
        });
    }

    public Command turnOffRightIntake() {
        return runOnce(() ->
        {
            setLeftRollerSpeed();
        });
    }
}
package frc.robot.subsystems.Intake;

import java.security.Key;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase{
    private final IntakeKraken mIntakeKraken = IntakeKraken.getInstance();
    private final IntakeStats ioStats = new IntakeStats();

    private DoublePublisher LExtendPosition;
    private DoublePublisher LExtendVelocity;
    private DoublePublisher LExtendAppliedVolts;
    private DoublePublisher LExtendSupplyCurrent;
    private DoublePublisher LExtendTorqueCurrent;
    private DoublePublisher LExtendTemp;

    private DoublePublisher RExtendPosition;
    private DoublePublisher RExtendVelocity;
    private DoublePublisher RExtendAppliedVolts;
    private DoublePublisher RExtendSupplyCurrent;
    private DoublePublisher RExtendTorqueCurrent;
    private DoublePublisher RExtendTemp;

    private DoublePublisher LRollerPosition;
    private DoublePublisher LRollerVelocity;
    private DoublePublisher LRollerAppliedVolts;
    private DoublePublisher LRollerSupplyCurrent;
    private DoublePublisher LRollerTorqueCurrent;
    private DoublePublisher LRollerTemp;

    private DoublePublisher RRollerPosition;
    private DoublePublisher RRollerVelocity;
    private DoublePublisher RRollerAppliedVolts;
    private DoublePublisher RRollerSupplyCurrent;
    private DoublePublisher RRollerTorqueCurrent;
    private DoublePublisher RRollerTemp;
    public Intake() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("HOT/Intake");

        LExtendVelocity = table.getDoubleTopic("LExtendVelocity-Motor (rot per s)").publish();
        LExtendPosition = table.getDoubleTopic("LExtendPosition-Motor (rot)").publish();
        LExtendSupplyCurrent = table.getDoubleTopic("LExtendSupply Current").publish();
        LExtendTorqueCurrent = table.getDoubleTopic("LExtendTorque Current").publish();
        LExtendTemp = table.getDoubleTopic("LExtendMotor Temp").publish();
        LExtendAppliedVolts = table.getDoubleTopic("LExtendApplied Volts").publish();

        RExtendVelocity = table.getDoubleTopic("RExtendVelocity-Motor (rot per b)").publish();
        RExtendPosition = table.getDoubleTopic("RExtendPosition-Motor (rot)").publish();
        RExtendSupplyCurrent = table.getDoubleTopic("RExtendSupply Current").publish();
        RExtendTorqueCurrent = table.getDoubleTopic("RExtendTorque Current").publish();
        RExtendTemp = table.getDoubleTopic("RExtendMotor Temp").publish();
        RExtendAppliedVolts = table.getDoubleTopic("RExtendApplied Volts").publish();

        LRollerVelocity = table.getDoubleTopic("LRollerVelocity-Motor (rot per a)").publish();
        LRollerPosition = table.getDoubleTopic("LRollerPosition-Motor (rot)").publish();
        LRollerSupplyCurrent = table.getDoubleTopic("LRollerSupply Current").publish();
        LRollerTorqueCurrent = table.getDoubleTopic("LRollerTorque Current").publish();
        LRollerTemp = table.getDoubleTopic("LRollerMotor Temp").publish();
        LRollerAppliedVolts = table.getDoubleTopic("LRollerApplied Volts").publish();

        RRollerVelocity = table.getDoubleTopic("RRollerVelocity-Motor (rot per c)").publish();
        RRollerPosition = table.getDoubleTopic("RRollerPosition-Motor (rot)").publish();
        RRollerSupplyCurrent = table.getDoubleTopic("RRollerSupply Current").publish();
        RRollerTorqueCurrent = table.getDoubleTopic("RRollerTorque Current").publish();
        RRollerTemp = table.getDoubleTopic("RRollerMotor Temp").publish();
        RRollerAppliedVolts = table.getDoubleTopic("RRollerApplied Volts").publish();
    }
    @Override
    public void periodic() {
        mIntakeKraken.updateStats(ioStats);
        UpdateTelemetry();
    }
    private double roundTo1Decimal(double number) {
        return Math.round(number * 10.0)/10.0;
    }

    private void UpdateTelemetry() {
    LExtendVelocity.set(roundTo1Decimal(ioStats.LExtendVelocityRpm));
    LExtendPosition.set(roundTo1Decimal(ioStats.LExtendPositionRads));
    LExtendSupplyCurrent.set(roundTo1Decimal(ioStats.LExtendSupplyCurrentAmps));
    LExtendTorqueCurrent.set(roundTo1Decimal(ioStats.LExtendTorqueCurrentAmps));
    LExtendAppliedVolts.set(roundTo1Decimal(ioStats.LExtendAppliedVolts));
    LExtendTemp.set(roundTo1Decimal(ioStats.LExtendTempCelsius));

    RExtendVelocity.set(roundTo1Decimal(ioStats.LExtendVelocityRpm));
    RExtendPosition.set(roundTo1Decimal(ioStats.LExtendPositionRads));
    RExtendSupplyCurrent.set(roundTo1Decimal(ioStats.LExtendSupplyCurrentAmps));
    RExtendTorqueCurrent.set(roundTo1Decimal(ioStats.LExtendTorqueCurrentAmps));
    RExtendAppliedVolts.set(roundTo1Decimal(ioStats.LExtendAppliedVolts));
    RExtendTemp.set(roundTo1Decimal(ioStats.LExtendTempCelsius));

    LRollerVelocity.set(roundTo1Decimal(ioStats.LExtendVelocityRpm));
    LRollerPosition.set(roundTo1Decimal(ioStats.LExtendPositionRads));
    LRollerSupplyCurrent.set(roundTo1Decimal(ioStats.LExtendSupplyCurrentAmps));
    LRollerTorqueCurrent.set(roundTo1Decimal(ioStats.LExtendTorqueCurrentAmps));
    LRollerAppliedVolts.set(roundTo1Decimal(ioStats.LExtendAppliedVolts));
    LRollerTemp.set(roundTo1Decimal(ioStats.LExtendTempCelsius));

    RRollerVelocity.set(roundTo1Decimal(ioStats.LExtendVelocityRpm));
    RRollerPosition.set(roundTo1Decimal(ioStats.LExtendPositionRads));
    RRollerSupplyCurrent.set(roundTo1Decimal(ioStats.LExtendSupplyCurrentAmps));
    RRollerTorqueCurrent.set(roundTo1Decimal(ioStats.LExtendTorqueCurrentAmps));
    RRollerAppliedVolts.set(roundTo1Decimal(ioStats.LExtendAppliedVolts));
    RRollerTemp.set(roundTo1Decimal(ioStats.LExtendTempCelsius));
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
    public Command turnOffLeftExpel() {
        return runOnce(() ->
          {
            setLeftIntakePosition(IntakeConstants.kLeftdeployedExpel);
        });
    }
    public Command turnOffRightExpel() {
        return runOnce(() ->
          {
            setRightIntakePosition(IntakeConstants.kRightdeployedExpel);
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
    class IntakeStats {
    //Stats for the Intake/Expel Motor  
    public boolean MotorConnected = true;    
    public double LExtendPositionRads = 0.0;
    public double LExtendVelocityRpm = 0.0;
    public double LExtendAppliedVolts = 0.0;
    public double LExtendSupplyCurrentAmps = 0.0;
    public double LExtendTorqueCurrentAmps = 0.0;
    public double LExtendTempCelsius = 0.0;   

    //Stats for the Intake/Expel Motor  
    public double RExtendPositionRads = 0.0;
    public double RExtendVelocityRpm = 0.0;
    public double RExtendAppliedVolts = 0.0;
    public double RExtendSupplyCurrentAmps = 0.0;
    public double RExtendTorqueCurrentAmps = 0.0;
    public double RExtendTempCelsius = 0.0; 
    
    //Stats for the Intake/Expel Motor     
    public double LRollerPositionRads = 0.0;
    public double LRollerVelocityRpm = 0.0;
    public double LRollerAppliedVolts = 0.0;
    public double LRollerSupplyCurrentAmps = 0.0;
    public double LRollerTorqueCurrentAmps = 0.0;
    public double LRollerTempCelsius = 0.0;      

    //Stats for the Intake/Expel Motor     
    public double RRollerPositionRads = 0.0;
    public double RRollerVelocityRpm = 0.0;
    public double RRollerAppliedVolts = 0.0;
    public double RRollerSupplyCurrentAmps = 0.0;
    public double RRollerTorqueCurrentAmps = 0.0;
    public double RRollerTempCelsius = 0.0;      
}
}
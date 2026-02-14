package frc.robot.subsystems.Intake;

import java.util.function.BooleanSupplier;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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

    private DoublePublisher LRollerPosition;
    private DoublePublisher LRollerVelocity;
    private DoublePublisher LRollerAppliedVolts;
    private DoublePublisher LRollerSupplyCurrent;
    private DoublePublisher LRollerTorqueCurrent;
    private DoublePublisher LRollerTemp;
 
    public Intake() {
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        NetworkTable table = inst.getTable("HOT/Intake");

        LExtendVelocity = table.getDoubleTopic("LExtendVelocity-Motor (rot per s)").publish();
        LExtendPosition = table.getDoubleTopic("LExtendPosition-Motor (rot)").publish();
        LExtendSupplyCurrent = table.getDoubleTopic("LExtendSupply Current").publish();
        LExtendTorqueCurrent = table.getDoubleTopic("LExtendTorque Current").publish();
        LExtendTemp = table.getDoubleTopic("LExtendMotor Temp").publish();
        LExtendAppliedVolts = table.getDoubleTopic("LExtendApplied Volts").publish();

        LRollerVelocity = table.getDoubleTopic("LRollerVelocity-Motor (rot per a)").publish();
        LRollerPosition = table.getDoubleTopic("LRollerPosition-Motor (rot)").publish();
        LRollerSupplyCurrent = table.getDoubleTopic("LRollerSupply Current").publish();
        LRollerTorqueCurrent = table.getDoubleTopic("LRollerTorque Current").publish();
        LRollerTemp = table.getDoubleTopic("LRollerMotor Temp").publish();
        LRollerAppliedVolts = table.getDoubleTopic("LRollerApplied Volts").publish();

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

        LRollerVelocity.set(roundTo1Decimal(ioStats.LRollerVelocityRpm));
        LRollerPosition.set(roundTo1Decimal(ioStats.LRollerPositionRads));
        LRollerSupplyCurrent.set(roundTo1Decimal(ioStats.LRollerSupplyCurrentAmps));
        LRollerTorqueCurrent.set(roundTo1Decimal(ioStats.LRollerTorqueCurrentAmps));
        LRollerAppliedVolts.set(roundTo1Decimal(ioStats.LRollerAppliedVolts));
        LRollerTemp.set(roundTo1Decimal(ioStats.LRollerTempCelsius));

    }

    /******************************************************************************************
     * COMMANDS 
     * @return a command 
     ******************************************************************************************/

    public Command extendLeftIntake() {
        return runOnce(() ->
        {
            mIntakeKraken.runPosition(IntakeConstants.kLeftDeployedPosition,1);
            mIntakeKraken.runVelocity(IntakeConstants.kLeftIntake,1);
        });
    }
     public Command retractLeftIntake() {
        return runOnce(() ->
        {
            mIntakeKraken.runPosition(IntakeConstants.kLeftRetractPosition,1);
            mIntakeKraken.stop();
        });
    }
    public Command expelLeftIntake() {
        return runOnce(() ->
          {
            mIntakeKraken.runPosition(IntakeConstants.kLeftDeployedPosition,1);
            mIntakeKraken.runVelocity(IntakeConstants.kLeftExpel,1);
        });
    } 
    public Command turnOffLeftExpel() {
        return runOnce(() ->
          {
            mIntakeKraken.stop();
        });
    }
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
    public double LRollerPositionRads = 0.0;
    public double LRollerVelocityRpm = 0.0;
    public double LRollerAppliedVolts = 0.0;
    public double LRollerSupplyCurrentAmps = 0.0;
    public double LRollerTorqueCurrentAmps = 0.0;
    public double LRollerTempCelsius = 0.0;
    public double LeftIntakeEncoderVelocity;
    public double LeftIntakeEncoderPosition;     
    }

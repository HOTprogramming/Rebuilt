package frc.robot.subsystems.Indexer;

import java.lang.annotation.Target;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Indexer.IndexKraken;

public class Index extends SubsystemBase{
    private final IndexStats ioStats = new IndexStats();
    private final IndexKraken m_IndexKraken = IndexKraken.getInstance();

    private DoublePublisher indexPosition;
    private DoublePublisher indexVelocity;
    private DoublePublisher indexAppliedVolts;
    private DoublePublisher indexSupplyCurrent;
    private DoublePublisher indexTorqueCurrent;
    private DoublePublisher indexTemp;


    class IndexStats {
    public boolean MotorConnected = true;
    public double indexPositionRads = 0.0;
    public double indexVelocityRpm = 0.0; 
    public double indexAppliedVolt = 0.0;
    public double indexSupplyCurrentAmps = 0.0;
    public double indexTorqueCurrentApls = 0.0;
    public double indexTempCelsius = 0.0;
    }
    public Index() {

            NetworkTableInstance inst = NetworkTableInstance.getDefault();

        NetworkTable table = inst.getTable("HOT/Twindexer Spindexer");

        indexVelocity = table.getDoubleTopic("spindexer Velocity").publish();
        indexPosition = table.getDoubleTopic("spindexer Position-Motor").publish();
        indexSupplyCurrent = table.getDoubleTopic("spindexer Supply Current").publish();
        indexTorqueCurrent = table.getDoubleTopic("spindexer Torque Current").publish();
        indexTemp = table.getDoubleTopic("spindexer Motor Temp").publish();
        indexAppliedVolts = table.getDoubleTopic("spindexer Applied Volts").publish();
}

@Override
public void periodic() {
    m_IndexKraken.updateStats(ioStats);
    UpdateTelemetry();
}    

    private void UpdateTelemetry() {
        indexVelocity.set(ioStats.indexVelocityRpm);
        indexPosition.set(ioStats.indexPositionRads);
        indexSupplyCurrent.set(ioStats.indexSupplyCurrentAmps);
        indexTorqueCurrent.set(ioStats.indexTorqueCurrentApls);
        indexTemp.set(ioStats.indexTempCelsius);
        indexAppliedVolts.set(ioStats.indexAppliedVolt);
    }

    private void kicker1(double targetPos){
        m_IndexKraken.runKick(targetPos);
    }
    private void kicker2(double targetPos){
        m_IndexKraken.runKick2(targetPos);
    }
    private void floor(double targetPos){
        m_IndexKraken.runFloor(targetPos);
    }
    private void load(double targetPos){
        m_IndexKraken.runLoad(targetPos);
    }
    private void off(double targetPos){
        m_IndexKraken.dontrun(targetPos);
    }

    public Command floorCommand() {
        return runOnce( () -> floor(IndexConstants.floorspeed));
    }
    public Command rightkickerCommand() {
        return runOnce( () -> kicker1(IndexConstants.kickerspeed));
    }
    public Command leftkickerCommand() {
        return runOnce( () -> kicker2(IndexConstants.kickerspeed));
    }
    public Command loadCommand() {
        return runOnce( () -> load(IndexConstants.loaderspeed));
    }
    public Command offCommand() {
        return runOnce( () -> off(IndexConstants.zerospeed));
    }
}
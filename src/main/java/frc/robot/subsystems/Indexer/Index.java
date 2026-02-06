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

    private DoublePublisher kickerLeftPosition;
    private DoublePublisher kickerLeftVelocity;
    private DoublePublisher kickerLeftAppliedVolts;
    private DoublePublisher kickerLeftSupplyCurrent;
    private DoublePublisher kickerLeftTorqueCurrent;
    private DoublePublisher kickerLeftTemp;
    private DoublePublisher kickerRightPosition;
    private DoublePublisher kickerRightVelocity;
    private DoublePublisher kickerRightAppliedVolts;
    private DoublePublisher kickerRightSupplyCurrent;
    private DoublePublisher kickerRightTorqueCurrent;
    private DoublePublisher kickerRightTemp;
    private DoublePublisher FloorPosition;
    private DoublePublisher FloorVelocity;
    private DoublePublisher FloorAppliedVolts;
    private DoublePublisher FloorSupplyCurrent;
    private DoublePublisher FloorTorqueCurrent;
    private DoublePublisher FloorTemp;
    private DoublePublisher LoadPosition;
    private DoublePublisher LoadVelocity;
    private DoublePublisher LoadAppliedVolts;
    private DoublePublisher LoadSupplyCurrent;
    private DoublePublisher LoadTorqueCurrent;
    private DoublePublisher LoadTemp;


    class IndexStats {
    public boolean MotorConnected = true;
    public double kickerLeftPositionRads = 0.0;
    public double kickerLeftVelocityRpm = 0.0; 
    public double kickerLeftAppliedVolt = 0.0;
    public double kickerLeftSupplyCurrentAmps = 0.0;
    public double kickerLeftTorqueCurrentApls = 0.0;
    public double kickerLeftTempCelsius = 0.0;
    public double kickerRightPositionRads = 0.0;
    public double kickerRightVelocityRpm = 0.0; 
    public double kickerRightAppliedVolt = 0.0;
    public double kickerRightSupplyCurrentAmps = 0.0;
    public double kickerRightTorqueCurrentApls = 0.0;
    public double kickerRightTempCelsius = 0.0;
    public double FloorPositionRads = 0.0;
    public double FloorVelocityRpm = 0.0; 
    public double FloorAppliedVolt = 0.0;
    public double FloorSupplyCurrentAmps = 0.0;
    public double FloorTorqueCurrentApls = 0.0;
    public double FloorTempCelsius = 0.0;
    public double LoadPositionRads = 0.0;
    public double LoadVelocityRpm = 0.0; 
    public double LoadAppliedVolt = 0.0;
    public double LoadSupplyCurrentAmps = 0.0;
    public double LoadTorqueCurrentApls = 0.0;
    public double LoadTempCelsius = 0.0;
    }
    public Index() {

            NetworkTableInstance inst = NetworkTableInstance.getDefault();

        NetworkTable table = inst.getTable("HOT/Indexer/Kicker");

        kickerLeftVelocity = table.getDoubleTopic("KickerLeft Velocity").publish();
        kickerLeftPosition = table.getDoubleTopic("KickerLeft Position-Motor").publish();
        kickerLeftSupplyCurrent = table.getDoubleTopic("KickerLeft Supply Current").publish();
        kickerLeftTorqueCurrent = table.getDoubleTopic("KickerLeft Torque Current").publish();
        kickerLeftTemp = table.getDoubleTopic("KickerLeft Motor Temp").publish();
        kickerLeftAppliedVolts = table.getDoubleTopic("KickerLeft Applied Volts").publish();
        kickerRightVelocity = table.getDoubleTopic("kickerRight Velocity").publish();
        kickerRightPosition = table.getDoubleTopic("kickerRight Position-Motor").publish();
        kickerRightSupplyCurrent = table.getDoubleTopic("kickerRight Supply Current").publish();
        kickerRightTorqueCurrent = table.getDoubleTopic("kickerRight Torque Current").publish();
        kickerRightTemp = table.getDoubleTopic("kickerRight Motor Temp").publish();
        kickerRightAppliedVolts = table.getDoubleTopic("kickerRight Applied Volts").publish();

        NetworkTable table2 = inst.getTable("HOT/Indexer/Floor");
        
        FloorVelocity = table2.getDoubleTopic("Floor Velocity").publish();
        FloorPosition = table2.getDoubleTopic("Floor Position-Motor").publish();
        FloorSupplyCurrent = table2.getDoubleTopic("Floor Supply Current").publish();
        FloorTorqueCurrent = table2.getDoubleTopic("Floor Torque Current").publish();
        FloorTemp = table2.getDoubleTopic("Floor Motor Temp").publish();
        FloorAppliedVolts = table2.getDoubleTopic("Floor Applied Volts").publish();

        NetworkTable table3 = inst.getTable("HOT/Indexer/Load");

        LoadVelocity = table3.getDoubleTopic("Load Velocity").publish();
        LoadPosition = table3.getDoubleTopic("Load Position-Motor").publish();
        LoadSupplyCurrent = table3.getDoubleTopic("Load Supply Current").publish();
        LoadTorqueCurrent = table3.getDoubleTopic("Load Torque Current").publish();
        LoadTemp = table3.getDoubleTopic("Load Motor Temp").publish();
        LoadAppliedVolts = table3.getDoubleTopic("Load Applied Volts").publish();

    }

@Override
public void periodic() {
    m_IndexKraken.updateStats(ioStats);
    UpdateTelemetry();
}    

    private void UpdateTelemetry() {
        kickerLeftVelocity.set(ioStats.kickerLeftVelocityRpm);
        kickerLeftPosition.set(ioStats.kickerLeftPositionRads);
        kickerLeftSupplyCurrent.set(ioStats.kickerLeftSupplyCurrentAmps);
        kickerLeftTorqueCurrent.set(ioStats.kickerLeftTorqueCurrentApls);
        kickerLeftTemp.set(ioStats.kickerLeftTempCelsius);
        kickerLeftAppliedVolts.set(ioStats.kickerLeftAppliedVolt);
        kickerRightVelocity.set(ioStats.kickerRightVelocityRpm);
        kickerRightPosition.set(ioStats.kickerRightPositionRads);
        kickerRightSupplyCurrent.set(ioStats.kickerRightSupplyCurrentAmps);
        kickerRightTorqueCurrent.set(ioStats.kickerRightTorqueCurrentApls);
        kickerRightTemp.set(ioStats.kickerRightTempCelsius);
        kickerRightAppliedVolts.set(ioStats.kickerRightAppliedVolt);
        FloorVelocity.set(ioStats.FloorVelocityRpm);
        FloorPosition.set(ioStats.FloorPositionRads);
        FloorSupplyCurrent.set(ioStats.FloorSupplyCurrentAmps);
        FloorTorqueCurrent.set(ioStats.FloorTorqueCurrentApls);
        FloorTemp.set(ioStats.FloorTempCelsius);
        FloorAppliedVolts.set(ioStats.FloorAppliedVolt);
        LoadVelocity.set(ioStats.LoadVelocityRpm);
        LoadPosition.set(ioStats.LoadPositionRads);
        LoadSupplyCurrent.set(ioStats.LoadSupplyCurrentAmps);
        LoadTorqueCurrent.set(ioStats.LoadTorqueCurrentApls);
        LoadTemp.set(ioStats.LoadTempCelsius);
        LoadAppliedVolts.set(ioStats.LoadAppliedVolt);
    }


    private void kickerstart(double targetPos){
        m_IndexKraken.startIndex(targetPos);
    }
    private void off(){
        m_IndexKraken.stop();
    }


    public Command indexOnCommand() {
        return runOnce( () -> kickerstart(IndexConstants.kickerSpeed));
    }
    public Command indexOffCommand() {
        return runOnce( () -> off());
    }

}
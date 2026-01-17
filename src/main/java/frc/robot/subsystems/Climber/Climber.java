package frc.robot.subsystems.Climber;

import java.security.Key;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climber extends SubsystemBase {
    private final ClimberStats ioStats = new ClimberStats();
    private final ClimberKraken m_ClimbKraken = ClimberKraken.getInstance();


    private DoublePublisher climberPosition;
    private DoublePublisher climberVelocity;
    private DoublePublisher climberAppliedVolts;
    private DoublePublisher climberSupplyCurrent;
    private DoublePublisher climberTorqueCurrent;
    private DoublePublisher climberTemp;


    class ClimberStats {
        public boolean MotorConnected = true;
        public double climberPositionRads = 0.0;
        public double climberVelocityRpm = 0.0; 
        public double climberAppliedVolt = 0.0;
        public double climberSupplyCurrentAmps = 0.0;
        public double climberTorqueCurrentApls = 0.0;
        public double climberTempCelsius = 0.0;
    }

    public Climber() {
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        NetworkTable table = inst.getTable("HOT/Climber");

        climberVelocity = table.getDoubleTopic("velocity").publish();
        climberPosition = table.getDoubleTopic("Position-Motor").publish();
        climberSupplyCurrent = table.getDoubleTopic("Supply Current").publish();
        climberTorqueCurrent = table.getDoubleTopic("Torque Current").publish();
        climberTemp = table.getDoubleTopic("Motor Temp").publish();
        climberAppliedVolts = table.getDoubleTopic("Applied Volts").publish();
    }

@Override
public void periodic() {
    m_ClimbKraken.updateStats(ioStats);
    UpdateTelemetry();
}    

    private void UpdateTelemetry() {
        climberVelocity.set(ioStats.climberVelocityRpm);
        climberPosition.set(ioStats.climberPositionRads);
        climberSupplyCurrent.set(ioStats.climberSupplyCurrentAmps);
        climberTorqueCurrent.set(ioStats.climberTorqueCurrentApls);
        climberTemp.set(ioStats.climberTempCelsius);
        climberAppliedVolts.set(ioStats.climberAppliedVolt);
    }

    private void runPosition(double targetPos){
        m_ClimbKraken.runPosition(targetPos);
    }


    public Command deployCommand() {
        return runOnce( () -> runPosition(ClimberConstants.deployPosition));
    }

    public Command retractCommand() {
        return runOnce ( () -> runPosition(ClimberConstants.retractPosition));
    }

    
}
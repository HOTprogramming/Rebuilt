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
    private DoublePublisher spinnerPosition;
    private DoublePublisher spinnerVelocity;
    private DoublePublisher spinnerAppliedVolts;
    private DoublePublisher spinnerSupplyCurrent;
    private DoublePublisher spinnerTorqueCurrent;
    private DoublePublisher spinnerTemp;
    


    class ClimberStats {
        public boolean MotorConnected = true;
        public double climberPositionRads = 0.0;
        public double climberVelocityRpm = 0.0; 
        public double climberAppliedVolt = 0.0;
        public double climberSupplyCurrentAmps = 0.0;
        public double climberTorqueCurrentApls = 0.0;
        public double climberTempCelsius = 0.0;
        public double spinnerPositionRads = 0.0;
        public double spinnerVelocityRpm = 0.0; 
        public double spinnerAppliedVolt = 0.0;
        public double spinnerSupplyCurrentAmps = 0.0;
        public double spinnerTorqueCurrentApls = 0.0;
        public double spinnerTempCelsius = 0.0;
    }

    public Climber() {
        
        NetworkTableInstance inst = NetworkTableInstance.getDefault();

        NetworkTable table = inst.getTable("HOT/Climber");

        climberVelocity = table.getDoubleTopic("Deploy Velocity").publish();
        climberPosition = table.getDoubleTopic("Deploy Position-Motor").publish();
        climberSupplyCurrent = table.getDoubleTopic("Deploy Supply Current").publish();
        climberTorqueCurrent = table.getDoubleTopic("Deploy Torque Current").publish();
        climberTemp = table.getDoubleTopic("Deploy Motor Temp").publish();
        climberAppliedVolts = table.getDoubleTopic("Deploy Applied Volts").publish();
        spinnerVelocity = table.getDoubleTopic("spin velocity").publish();
        spinnerPosition = table.getDoubleTopic("spin Position-Motor").publish();
        spinnerSupplyCurrent = table.getDoubleTopic("spin Supply Current").publish();
        spinnerTorqueCurrent = table.getDoubleTopic("spin Torque Current").publish();
        spinnerTemp = table.getDoubleTopic("spin Motor Temp").publish();
        spinnerAppliedVolts = table.getDoubleTopic("spin Applied Volts").publish();
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
        spinnerVelocity.set(ioStats.spinnerVelocityRpm);
        spinnerPosition.set(ioStats.spinnerPositionRads);
        spinnerSupplyCurrent.set(ioStats.spinnerSupplyCurrentAmps);
        spinnerTorqueCurrent.set(ioStats.spinnerTorqueCurrentApls);
        spinnerTemp.set(ioStats.spinnerTempCelsius);
        spinnerAppliedVolts.set(ioStats.spinnerAppliedVolt);
    }

    private void runPosition(double targetPos){
        m_ClimbKraken.runPosition(targetPos);
    }
    private void spinrunPosition(double targetPos){
        m_ClimbKraken.spinrunPosition(targetPos);
    }

    public Command deployCommand() {
        return runOnce( () -> runPosition(ClimberConstants.deployPosition));
    }
    public Command retractCommand() {
        return runOnce ( () -> runPosition(ClimberConstants.retractPosition));
    }
    public Command rotateCommand() {
        return runOnce ( () -> spinrunPosition(ClimberConstants.spinPosition));
    }
    public Command spinhomeCommand() {
        return runOnce ( () -> spinrunPosition(ClimberConstants.spinhomePosition));
    }
    public Command latchCommand() {
        return runOnce ( () -> spinrunPosition(ClimberConstants.lockPosition));
    }
    public Command LV1Command() {
        return runOnce ( () -> spinrunPosition(ClimberConstants.lvlonePosition));
    }
}
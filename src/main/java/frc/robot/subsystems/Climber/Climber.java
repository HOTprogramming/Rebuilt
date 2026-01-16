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

//deploy - stow
//extend - retract
//latch - unlatch
//asend - descend
//hang
//failsafe desend

private DoublePublisher climberPosition;
private DoublePublisher climberVelocity;
private DoublePublisher climberAppliedVolts;
private DoublePublisher climberSupplyCurrent;
private DoublePublisher climberTorqueCurrent;
private DoublePublisher climberTemp;
private StringPublisher climberState;

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
        
        NetworkTable inst = NetworkTableInstance.getDefault();

        NetworkTable table = inst.getTable("HOT/Climber");

        climberVelocity = table.getDoubleTopic("velocity").publish();
        climberPosition = table.getDoubleTopic("Position-Motor").publish();
        climberSupplyCurrent = table.getDoubleTopic("Supply Current").publish();
        climberTorqueCurrent = table.getDoubleTopic("Torque Current").publish();
        climberTemp = table.getDoubleTopic("Motor Temp").publish();
        climberAppliedVolts = table.getDoubleTopic("Applied Volts").publish();
        climberState = table.getStringTopic("State").publish();
        }

    @Override
    public void periodic() {
        m_ClimbKraken.
    }    
    

    public Command climbCommand() {
        return runOnce( () -> deployClimber().withName("Deploy Climber"));
    }
    
    public Command reachCommand() {
        return runOnce( () -> reachClimber().withName("reach")); //reach for the first bar
    }
    
    public Command latchCommand() {
        return runOnce( () -> latchClimber().withName("latch")); //latch on to the bar to begin ascention
    }

    public Command ascendCommand() {
        return runOnce( () -> ascendClimber().withName("ascend"));
    }

    public Command parkCommand() {
        return runOnce( () -> parkClimber().withName("park")); //to keep the robot hanging without doing anything
    }
    
    public Command descendCommand() {
        return runOnce( () -> descendClimber().withName("lower"));
    }

        public Command detachCommand() {
        return runOnce( () -> detachClimber().withName("detach"));
    }

    public Command undeployCommand() {
        return runOnce( () -> undeployClimber().withName("undeploy"));
    }



}
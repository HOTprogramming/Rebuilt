package frc.robot.subsystems.Climber;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Climb extends SubsystemBase {

//deploy
//reach
//latch
//asend
//park (detect when climbed)
//descend
//undeploy
//failsafe desend


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

    public Command reachCommand() {
        return runOnce( () -> reachClimber().withName("reach"));
    }
}
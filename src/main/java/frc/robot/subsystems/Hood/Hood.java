package frc.robot.subsystems.Hood;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hood extends SubsystemBase {
    private final hoodStats motorStats = new hoodStats();
    private final HoodKraken mHoodKraken = HoodKraken.getInstance();

    /* Network Table entries */
    private DoublePublisher hoodPosition;
    private DoublePublisher hoodVelocity;
    private DoublePublisher hoodAppliedVolts;
    private DoublePublisher hoodSupplyCurrent;
    private DoublePublisher hoodTorqueCurrent;
    private DoublePublisher hoodTemp;
    private DoublePublisher hoodGoal;

    /* Initializes the hood stats */
    class hoodStats {
        public boolean MotorConnected = true;    
        public double hoodPositionRads = 0.0;
        public double hoodVelocityRpm = 0.0;
        public double hoodAppliedVolts = 0.0;
        public double hoodSupplyCurrentAmps = 0.0;
        public double hoodTorqueCurrentAmps = 0.0;
        public double hoodTempCelsius = 0.0;
    }
    
    // Constructor for Hood Object //
    public Hood() {
        NetworkTableInstance instance = NetworkTableInstance.getDefault();

        NetworkTable table = instance.getTable("HOT/Hood");

        /* Creates the Display for the Network Tables */
        hoodPosition = table.getDoubleTopic("Hood: Velocity (RPS)").publish();
        hoodVelocity = table.getDoubleTopic("Hood: Position-Motor (ROT)").publish();
        hoodAppliedVolts = table.getDoubleTopic("Hood: Applied Volts").publish();
        hoodSupplyCurrent = table.getDoubleTopic("Hood: Supply Current").publish();
        hoodTorqueCurrent = table.getDoubleTopic("Hood: Torque Current").publish();
        hoodTemp = table.getDoubleTopic("Hood: Motor Temp").publish();
        hoodGoal = table.getDoubleTopic("Hood: Goal Position").publish();
    }
    
    public void setHoodPosition(double mHoodPosition) {
    // Setter //
        mHoodKraken.runPosition(mHoodPosition);
    }
    
    @Override  
    /* This method will be called once per scheduler run */
    public void periodic() {
    //go update the signal data 
      mHoodKraken.updateStats(motorStats);
      
      UpdateTelemetry();
    }

    /************************************************
   * Update the Network Tables with Motor Statistics 
   ************************************************/
    private void UpdateTelemetry() {
        hoodVelocity.set(motorStats.hoodVelocityRpm);
        hoodPosition.set(motorStats.hoodPositionRads);
        hoodSupplyCurrent.set(motorStats.hoodSupplyCurrentAmps);
        hoodTorqueCurrent.set(motorStats.hoodTorqueCurrentAmps);
        hoodAppliedVolts.set(motorStats.hoodAppliedVolts);
        hoodTemp.set(motorStats.hoodTempCelsius);
    }

    /**************************************************************
  * Commands 
  **************************************************************/
  /**
   * Set hood to a position of 0
   *
   * @return a command
   */
    public Command goTo0() {
        return runOnce(
            () -> {
            /* one-time action goes here */
            hoodGoal.set(HoodConstants.hoodPosition0);
            setHoodPosition(HoodConstants.hoodPosition0);
        });
    }
    /**
   * Set hood to a position of 15
   * 
   * @return a command
   */
    public Command goTo15() {
        return runOnce(
            () -> {
            /* one-time action goes here */
            hoodGoal.set(HoodConstants.hoodPosition1);
            setHoodPosition(HoodConstants.hoodPosition1);
        });
    }
    /**
   * Set hood to a position of 30
   * 
   * @return a command
   */
    public Command goTo30() {
        return runOnce(
            () -> {
            /* one-time action goes here */
            hoodGoal.set(HoodConstants.hoodPosition2);
            setHoodPosition(HoodConstants.hoodPosition2);
        });
    }
    /**
   * Set hood to a position of 45
   * 
   * @return a command
   */
    public Command goTo45() {
        return runOnce(
            () -> {
            /* one-time action goes here */
            hoodGoal.set(HoodConstants.hoodPosition3);
            setHoodPosition(HoodConstants.hoodPosition3);
        });
    }
    /**
   * Set hood to a position of 60
   * 
   * @return a command
   */
    public Command goTo60() {
        return runOnce(
            () -> {
            /* one-time action goes here */
            hoodGoal.set(HoodConstants.hoodPosition4);
            setHoodPosition(HoodConstants.hoodPosition4);
        });
    }
}

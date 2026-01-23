package frc.robot.subsystems.Shooter;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
  private final shooterStats motorStats = new shooterStats();
  private final ShooterKraken mShooterKraken = ShooterKraken.getInstance();

/* Network Table entries */
  private DoublePublisher shooterPosition;
  private DoublePublisher shooterVelocity;
  private DoublePublisher shooterAppliedVolts;
  private DoublePublisher shooterSupplyCurrent;
  private DoublePublisher shooterTorqueCurrent;
  private DoublePublisher shooterTemp;

  private DoublePublisher shooterPositionSub;
  private DoublePublisher shooterVelocitySub;
  private DoublePublisher shooterAppliedVoltsSub;
  private DoublePublisher shooterSupplyCurrentSub;
  private DoublePublisher shooterTorqueCurrentSub;
  private DoublePublisher shooterTempSub;

  private DoublePublisher shooterPositionSubSub;
  private DoublePublisher shooterVelocitySubSub;
  private DoublePublisher shooterAppliedVoltsSubSub;
  private DoublePublisher shooterSupplyCurrentSubSub;
  private DoublePublisher shooterTorqueCurrentSubSub;
  private DoublePublisher shooterTempSubSub;

  /* Initializes the shooter stats */
  class shooterStats {
    public boolean MotorConnected = true;    
    public double shooterPositionRads = 0.0;
    public double shooterVelocityRpm = 0.0;
    public double shooterAppliedVolts = 0.0;
    public double shooterSupplyCurrentAmps = 0.0;
    public double shooterTorqueCurrentAmps = 0.0;
    public double shooterTempCelsius = 0.0;

    public boolean MotorConnectedSub = true;    
    public double shooterPositionRadsSub = 0.0;
    public double shooterVelocityRpmSub = 0.0;
    public double shooterAppliedVoltsSub = 0.0;
    public double shooterSupplyCurrentAmpsSub = 0.0;
    public double shooterTorqueCurrentAmpsSub = 0.0;
    public double shooterTempCelsiusSub = 0.0;  

    public boolean MotorConnectedSubSub = true;    
    public double shooterPositionRadsSubSub = 0.0;
    public double shooterVelocityRpmSubSub = 0.0;
    public double shooterAppliedVoltsSubSub = 0.0;
    public double shooterSupplyCurrentAmpsSubSub = 0.0;
    public double shooterTorqueCurrentAmpsSubSub = 0.0;
    public double shooterTempCelsiusSubSub = 0.0;  
  }

//Constructor for Shooter Object//
    public Shooter() {
      NetworkTableInstance instance = NetworkTableInstance.getDefault();

      NetworkTable table = instance.getTable("HOT/Shooter");
      
      /* Creates the Display for the Network Tables */
      shooterPosition = table.getDoubleTopic("Master: Velocity (RPS)").publish();
      shooterVelocity = table.getDoubleTopic("Master: Position-Motor (ROT)").publish();
      shooterAppliedVolts = table.getDoubleTopic("Master: Applied Volts").publish();
      shooterSupplyCurrent = table.getDoubleTopic("Master: Supply Current").publish();
      shooterTorqueCurrent = table.getDoubleTopic("Master: Torque Current").publish();
      shooterTemp = table.getDoubleTopic("Master: Motor Temp").publish();

      shooterPositionSub = table.getDoubleTopic("Follower 1: Velocity (RPS)").publish();
      shooterVelocitySub = table.getDoubleTopic("Follower 1: Position-Motor (ROT)").publish();
      shooterAppliedVoltsSub = table.getDoubleTopic("Follower 1: Applied Volts").publish();
      shooterSupplyCurrentSub = table.getDoubleTopic("Follower 1: Supply Current").publish();
      shooterTorqueCurrentSub = table.getDoubleTopic("Follower 1: Torque Current").publish();
      shooterTempSub = table.getDoubleTopic("Follower 1: Motor Temp").publish();

      shooterPositionSubSub = table.getDoubleTopic("Follower 2: Velocity (RPS)").publish();
      shooterVelocitySubSub = table.getDoubleTopic("Follower 2: Position-Motor (ROT)").publish();
      shooterAppliedVoltsSubSub = table.getDoubleTopic("Follower 2: Applied Volts").publish();
      shooterSupplyCurrentSubSub = table.getDoubleTopic("Follower 2: Supply Current").publish();
      shooterTorqueCurrentSubSub = table.getDoubleTopic("Follower 2: Torque Current").publish();
      shooterTempSubSub = table.getDoubleTopic("Follower 2: Motor Temp").publish();
    }

    //Setter//
    private void setShooterSpeed(int shooterSpeed) {
      mShooterKraken.runVelocity(shooterSpeed);
    }

    //Getter//
    public int getShooterSpeed() {
      return ShooterConstants.kShooterSpeed;
    }
  
    @Override  
    /* This method will be called once per scheduler run */
    public void periodic() {
    //go update the signal data 
      mShooterKraken.updateStats(motorStats);
      
      UpdateTelemetry();
    }
  /************************************************
   * Update the Network Tables with Motor Statistics 
   ************************************************/
  private void UpdateTelemetry() {
    shooterVelocity.set(motorStats.shooterVelocityRpm);
    shooterPosition.set(motorStats.shooterPositionRads);
    shooterSupplyCurrent.set(motorStats.shooterSupplyCurrentAmps);
    shooterTorqueCurrent.set(motorStats.shooterTorqueCurrentAmps);
    shooterAppliedVolts.set(motorStats.shooterAppliedVolts);
    shooterTemp.set(motorStats.shooterTempCelsius);

    shooterVelocitySub.set(motorStats.shooterVelocityRpmSub);
    shooterPositionSub.set(motorStats.shooterPositionRadsSub);
    shooterSupplyCurrentSub.set(motorStats.shooterSupplyCurrentAmpsSub);
    shooterTorqueCurrentSub.set(motorStats.shooterTorqueCurrentAmpsSub);
    shooterAppliedVoltsSub.set(motorStats.shooterAppliedVoltsSub);
    shooterTempSub.set(motorStats.shooterTempCelsiusSub);

    shooterVelocitySubSub.set(motorStats.shooterVelocityRpmSubSub);
    shooterPositionSubSub.set(motorStats.shooterPositionRadsSubSub);
    shooterSupplyCurrentSubSub.set(motorStats.shooterSupplyCurrentAmpsSubSub);
    shooterTorqueCurrentSubSub.set(motorStats.shooterTorqueCurrentAmpsSubSub);
    shooterAppliedVoltsSubSub.set(motorStats.shooterAppliedVoltsSubSub);
    shooterTempSubSub.set(motorStats.shooterTempCelsiusSubSub);
  }

  /**************************************************************
  * Commands 
  **************************************************************/
  /**
   * Set shooter to idle speed
   *
   * @return a command
   */
  public Command goToIdle() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          setShooterSpeed(ShooterConstants.kIdleSpeed);
        });
    }
  /**
   * Set shooter to shoot speed
   * 
   * @return a command
   */
  public Command goToShoot() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          setShooterSpeed(ShooterConstants.kShooterSpeed);
        });
    }
}
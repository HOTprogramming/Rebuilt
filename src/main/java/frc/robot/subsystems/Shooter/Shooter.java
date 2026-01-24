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

  private DoublePublisher shooterPositionOther;
  private DoublePublisher shooterVelocityOther;
  private DoublePublisher shooterAppliedVoltsOther;
  private DoublePublisher shooterSupplyCurrentOther;
  private DoublePublisher shooterTorqueCurrentOther;
  private DoublePublisher shooterTempOther;

  /* Initializes the shooter stats */
  class shooterStats {
    public boolean MotorConnected = true;    
    public double shooterPositionRads = 0.0;
    public double shooterVelocityRpm = 0.0;
    public double shooterAppliedVolts = 0.0;
    public double shooterSupplyCurrentAmps = 0.0;
    public double shooterTorqueCurrentAmps = 0.0;
    public double shooterTempCelsius = 0.0;

    public boolean MotorConnectedOther = true;    
    public double shooterPositionRadsOther = 0.0;
    public double shooterVelocityRpmOther = 0.0;
    public double shooterAppliedVoltsOther = 0.0;
    public double shooterSupplyCurrentAmpsOther = 0.0;
    public double shooterTorqueCurrentAmpsOther = 0.0;
    public double shooterTempCelsiusOther = 0.0; 
  }

//Constructor for Shooter Object//
    public Shooter() {
      NetworkTableInstance instance = NetworkTableInstance.getDefault();

      NetworkTable table = instance.getTable("HOT/Shooter");
      
      /* Creates the Display for the Network Tables */
      shooterPosition = table.getDoubleTopic("Motor 1: Velocity (RPS)").publish();
      shooterVelocity = table.getDoubleTopic("Motor 1: Position-Motor (ROT)").publish();
      shooterAppliedVolts = table.getDoubleTopic("Motor 1: Applied Volts").publish();
      shooterSupplyCurrent = table.getDoubleTopic("Motor 1: Supply Current").publish();
      shooterTorqueCurrent = table.getDoubleTopic("Motor 1: Torque Current").publish();
      shooterTemp = table.getDoubleTopic("Motor 1: Motor Temp").publish();

      shooterPositionOther = table.getDoubleTopic("Motor 2: Velocity (RPS)").publish();
      shooterVelocityOther = table.getDoubleTopic("Motor 2: Position-Motor (ROT)").publish();
      shooterAppliedVoltsOther = table.getDoubleTopic("Motor 2: Applied Volts").publish();
      shooterSupplyCurrentOther = table.getDoubleTopic("Motor 2: Supply Current").publish();
      shooterTorqueCurrentOther = table.getDoubleTopic("Motor 2: Torque Current").publish();
      shooterTempOther = table.getDoubleTopic("Motor 2: Motor Temp").publish();
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

    shooterVelocityOther.set(motorStats.shooterVelocityRpmOther);
    shooterPositionOther.set(motorStats.shooterPositionRadsOther);
    shooterSupplyCurrentOther.set(motorStats.shooterSupplyCurrentAmpsOther);
    shooterTorqueCurrentOther.set(motorStats.shooterTorqueCurrentAmpsOther);
    shooterAppliedVoltsOther.set(motorStats.shooterAppliedVoltsOther);
    shooterTempOther.set(motorStats.shooterTempCelsiusOther);
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
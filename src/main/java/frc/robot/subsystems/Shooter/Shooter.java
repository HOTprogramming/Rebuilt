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

  private DoublePublisher hoodPosition;
  private DoublePublisher hoodVelocity;
  private DoublePublisher hoodAppliedVolts;
  private DoublePublisher hoodSupplyCurrent;
  private DoublePublisher hoodTorqueCurrent;
  private DoublePublisher hoodTemp;
  private DoublePublisher hoodGoal;

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

    public double hoodPositionRads = 0.0;
    public double hoodVelocityRpm = 0.0;
    public double hoodAppliedVolts = 0.0;
    public double hoodSupplyCurrentAmps = 0.0;
    public double hoodTorqueCurrentAmps = 0.0;
    public double hoodTempCelsius = 0.0;
  }

//Constructor for Shooter Object//
    public Shooter() {
      NetworkTableInstance instance = NetworkTableInstance.getDefault();

      NetworkTable table = instance.getTable("HOT/Shooter");
      
      /* Creates the Display for the Network Tables */
      shooterPosition = table.getDoubleTopic("Shooter 1: Position-Motor (ROT)").publish();
      shooterVelocity = table.getDoubleTopic("Shooter 1: Velocity (RPS)").publish();
      shooterAppliedVolts = table.getDoubleTopic("Shooter 1: Applied Volts").publish();
      shooterSupplyCurrent = table.getDoubleTopic("Shooter 1: Supply Current").publish();
      shooterTorqueCurrent = table.getDoubleTopic("Shooter 1: Torque Current").publish();
      shooterTemp = table.getDoubleTopic("Shooter 1: Motor Temp").publish();

      shooterPositionOther = table.getDoubleTopic("Shooter 2: Position-Motor (ROT)").publish();
      shooterVelocityOther = table.getDoubleTopic("Shooter 2: Velocity (RPS)").publish();
      shooterAppliedVoltsOther = table.getDoubleTopic("Shooter 2: Applied Volts").publish();
      shooterSupplyCurrentOther = table.getDoubleTopic("Shooter 2: Supply Current").publish();
      shooterTorqueCurrentOther = table.getDoubleTopic("Shooter 2: Torque Current").publish();
      shooterTempOther = table.getDoubleTopic("Shooter 2: Motor Temp").publish();

      hoodPosition = table.getDoubleTopic("Hood: Position-Motor (ROT)").publish();
      hoodVelocity = table.getDoubleTopic("Hood: Velocity (RPS)").publish();
      hoodAppliedVolts = table.getDoubleTopic("Hood: Applied Volts").publish();
      hoodSupplyCurrent = table.getDoubleTopic("Hood: Supply Current").publish();
      hoodTorqueCurrent = table.getDoubleTopic("Hood: Torque Current").publish();
      hoodTemp = table.getDoubleTopic("Hood: Motor Temp").publish();
      hoodGoal = table.getDoubleTopic("Hood: Goal Position").publish();
    }

    //Shooter - Getter//
    public double getShooterSpeed() {
      return motorStats.shooterVelocityRpm;
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
   * Set shooter to idle speed
   *
   * @return a command
   */
  public Command goToIdle() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          mShooterKraken.runVelocity(ShooterConstants.kIdleSpeed);
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
          mShooterKraken.runVelocity(ShooterConstants.kShooterSpeed);
        });
    }
  /**
   * Set hood to a position of 0
   *
   * @return a command
   */
    public Command goTo0() {
        return runOnce(
            () -> {
            /* one-time action goes here */
            hoodGoal.set(ShooterConstants.hoodPosition0);
            mShooterKraken.runPosition(ShooterConstants.hoodPosition0);
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
            hoodGoal.set(ShooterConstants.hoodPosition1);
            mShooterKraken.runPosition(ShooterConstants.hoodPosition1);
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
            hoodGoal.set(ShooterConstants.hoodPosition2);
            mShooterKraken.runPosition(ShooterConstants.hoodPosition2);
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
            hoodGoal.set(ShooterConstants.hoodPosition3);
            mShooterKraken.runPosition(ShooterConstants.hoodPosition3);
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
            hoodGoal.set(ShooterConstants.hoodPosition4);
            mShooterKraken.runPosition(ShooterConstants.hoodPosition4);
        });
    }
}
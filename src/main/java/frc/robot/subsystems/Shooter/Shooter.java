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

  private DoublePublisher shooterPositionSub1;
  private DoublePublisher shooterVelocitySub1;
  private DoublePublisher shooterAppliedVoltsSub1;
  private DoublePublisher shooterSupplyCurrentSub1;
  private DoublePublisher shooterTorqueCurrentSub1;
  private DoublePublisher shooterTempSub1;

  private DoublePublisher shooterPositionSub2;
  private DoublePublisher shooterVelocitySub2;
  private DoublePublisher shooterAppliedVoltsSub2;
  private DoublePublisher shooterSupplyCurrentSub2;
  private DoublePublisher shooterTorqueCurrentSub2;
  private DoublePublisher shooterTempSub2;

  private DoublePublisher shooterPositionSub3;
  private DoublePublisher shooterVelocitySub3;
  private DoublePublisher shooterAppliedVoltsSub3;
  private DoublePublisher shooterSupplyCurrentSub3;
  private DoublePublisher shooterTorqueCurrentSub3;
  private DoublePublisher shooterTempSub3;

  private DoublePublisher hoodPosition;
  private DoublePublisher hoodVelocity;
  private DoublePublisher hoodAppliedVolts;
  private DoublePublisher hoodSupplyCurrent;
  private DoublePublisher hoodTorqueCurrent;
  private DoublePublisher hoodTemp;

  /* Initializes the shooter stats */
  class shooterStats {
    public boolean MotorConnected = true;    
    public double shooterPositionRads = 0.0;
    public double shooterVelocityRpm = 0.0;
    public double shooterAppliedVolts = 0.0;
    public double shooterSupplyCurrentAmps = 0.0;
    public double shooterTorqueCurrentAmps = 0.0;
    public double shooterTempCelsius = 0.0;

    public boolean MotorConnectedSub1 = true;    
    public double shooterPositionRadsSub1 = 0.0;
    public double shooterVelocityRpmSub1 = 0.0;
    public double shooterAppliedVoltsSub1 = 0.0;
    public double shooterSupplyCurrentAmpsSub1 = 0.0;
    public double shooterTorqueCurrentAmpsSub1 = 0.0;
    public double shooterTempCelsiusSub1 = 0.0;

    public boolean MotorConnectedSub2 = true;    
    public double shooterPositionRadsSub2 = 0.0;
    public double shooterVelocityRpmSub2 = 0.0;
    public double shooterAppliedVoltsSub2 = 0.0;
    public double shooterSupplyCurrentAmpsSub2 = 0.0;
    public double shooterTorqueCurrentAmpsSub2 = 0.0;
    public double shooterTempCelsiusSub2 = 0.0;

    public boolean MotorConnectedSub3 = true;    
    public double shooterPositionRadsSub3 = 0.0;
    public double shooterVelocityRpmSub3 = 0.0;
    public double shooterAppliedVoltsSub3 = 0.0;
    public double shooterSupplyCurrentAmpsSub3 = 0.0;
    public double shooterTorqueCurrentAmpsSub3 = 0.0;
    public double shooterTempCelsiusSub3 = 0.0;

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
    shooterPosition = table.getDoubleTopic("Shooter Master: Position-Motor (ROT)").publish();
    shooterVelocity = table.getDoubleTopic("Shooter Master: Velocity (RPS)").publish();
    shooterAppliedVolts = table.getDoubleTopic("Shooter Master: Applied Volts").publish();
    shooterSupplyCurrent = table.getDoubleTopic("Shooter Master: Supply Current").publish();
    shooterTorqueCurrent = table.getDoubleTopic("Shooter Master: Torque Current").publish();
    shooterTemp = table.getDoubleTopic("Shooter Master: Motor Temp").publish();

    shooterPositionSub1 = table.getDoubleTopic("Shooter Sub 1: Position-Motor (ROT)").publish();
    shooterVelocitySub1 = table.getDoubleTopic("Shooter Sub 1: Velocity (RPS)").publish();
    shooterAppliedVoltsSub1 = table.getDoubleTopic("Shooter Sub 1: Applied Volts").publish();
    shooterSupplyCurrentSub1 = table.getDoubleTopic("Shooter Sub 1: Supply Current").publish();
    shooterTorqueCurrentSub1 = table.getDoubleTopic("Shooter Sub 1: Torque Current").publish();
    shooterTempSub1 = table.getDoubleTopic("Shooter Sub 1: Motor Temp").publish();

    shooterPositionSub2 = table.getDoubleTopic("Shooter Sub 2: Position-Motor (ROT)").publish();
    shooterVelocitySub2 = table.getDoubleTopic("Shooter Sub 2: Velocity (RPS)").publish();
    shooterAppliedVoltsSub2 = table.getDoubleTopic("Shooter Sub 2: Applied Volts").publish();
    shooterSupplyCurrentSub2 = table.getDoubleTopic("Shooter Sub 2: Supply Current").publish();
    shooterTorqueCurrentSub2 = table.getDoubleTopic("Shooter Sub 2: Torque Current").publish();
    shooterTempSub2 = table.getDoubleTopic("Shooter Sub 2: Motor Temp").publish();

    shooterPositionSub3 = table.getDoubleTopic("Shooter Sub 3: Position-Motor (ROT)").publish();
    shooterVelocitySub3 = table.getDoubleTopic("Shooter Sub 3: Velocity (RPS)").publish();
    shooterAppliedVoltsSub3 = table.getDoubleTopic("Shooter Sub 3: Applied Volts").publish();
    shooterSupplyCurrentSub3 = table.getDoubleTopic("Shooter Sub 3: Supply Current").publish();
    shooterTorqueCurrentSub3 = table.getDoubleTopic("Shooter Sub 3: Torque Current").publish();
    shooterTempSub3 = table.getDoubleTopic("Shooter Sub 3: Motor Temp").publish();

    hoodPosition = table.getDoubleTopic("Hood: Position-Motor (ROT)").publish();
    hoodVelocity = table.getDoubleTopic("Hood: Velocity (RPS)").publish();
    hoodAppliedVolts = table.getDoubleTopic("Hood: Applied Volts").publish();
    hoodSupplyCurrent = table.getDoubleTopic("Hood: Supply Current").publish();
    hoodTorqueCurrent = table.getDoubleTopic("Hood: Torque Current").publish();
    hoodTemp = table.getDoubleTopic("Hood: Motor Temp").publish();
    }

    //Shooter - get Target Speed//
    public double getTargetShootSpeed() {
      return ShooterConstants.kTargetShootSpeed;
    }

    //Hood - get Target Angle//
    public double getTargetHoodAngle() {
      return ShooterConstants.kTargetHoodAngle;
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

    shooterVelocitySub1.set(motorStats.shooterVelocityRpmSub1);
    shooterPositionSub1.set(motorStats.shooterPositionRadsSub1);
    shooterSupplyCurrentSub1.set(motorStats.shooterSupplyCurrentAmpsSub1);
    shooterTorqueCurrentSub1.set(motorStats.shooterTorqueCurrentAmpsSub1);
    shooterAppliedVoltsSub1.set(motorStats.shooterAppliedVoltsSub1);
    shooterTempSub1.set(motorStats.shooterTempCelsiusSub1);

    shooterVelocitySub2.set(motorStats.shooterVelocityRpmSub2);
    shooterPositionSub2.set(motorStats.shooterPositionRadsSub2);
    shooterSupplyCurrentSub2.set(motorStats.shooterSupplyCurrentAmpsSub2);
    shooterTorqueCurrentSub2.set(motorStats.shooterTorqueCurrentAmpsSub2);
    shooterAppliedVoltsSub2.set(motorStats.shooterAppliedVoltsSub2);
    shooterTempSub2.set(motorStats.shooterTempCelsiusSub2);

    shooterVelocitySub3.set(motorStats.shooterVelocityRpmSub3);
    shooterPositionSub3.set(motorStats.shooterPositionRadsSub3);
    shooterSupplyCurrentSub3.set(motorStats.shooterSupplyCurrentAmpsSub3);
    shooterTorqueCurrentSub3.set(motorStats.shooterTorqueCurrentAmpsSub3);
    shooterAppliedVoltsSub3.set(motorStats.shooterAppliedVoltsSub3);
    shooterTempSub3.set(motorStats.shooterTempCelsiusSub3);

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
   * Turn off shooter motor
   * 
   * @return a command
   */
  public Command shooterOff() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          mShooterKraken.stop();
        });
    }  
  /**
   * Raise shooter speed by 2 rps
   * 
   * @return a command
   */
  public Command shootSpeedUp() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          ShooterConstants.kTargetShootSpeed += 2;
          mShooterKraken.runVelocity(ShooterConstants.kTargetShootSpeed); 
        });
    }
    /**
   * Lower shooter speed by 2 rps
   * 
   * @return a command
   */
  public Command shootSpeedDown() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          ShooterConstants.kTargetShootSpeed -= 2;
          mShooterKraken.runVelocity(ShooterConstants.kTargetShootSpeed);
        });
    }
  /**
   * Return true if shooter flywheel is up to speed
   * 
   * @return a command
   */
  public Boolean readyToShoot() {
    return (getShooterSpeed() >= ShooterConstants.kShooterSpeed -ShooterConstants.kShooterError);
  }

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
   * Change the hood angle
   *
   * @return a command
   */
    public Command setHoodAngle(double angle) {
      return runOnce(
        () -> {
          /* one-time action goes here */
          hoodPosition.set(angle);
          mShooterKraken.runPosition(angle);
        });
    }
    /**
     * Raise hood angle by 15
     * 
     * @return a command
     */
    public Command hoodAngleUp() {
      return runOnce(
        () -> {
          /* one-time action goes here */
          ShooterConstants.kTargetHoodAngle += 0.1;
          mShooterKraken.runPosition(ShooterConstants.kTargetHoodAngle);
        });
    }
    /**
     * Lower hood angle by 15
     * 
     * @return a command
     */
    public Command hoodAngleDown() {
      return runOnce(
        () -> {
          /* one-time action goes here */
          ShooterConstants.kTargetHoodAngle -= 0.1;
          mShooterKraken.runPosition(ShooterConstants.kTargetHoodAngle);
        });
    }
}
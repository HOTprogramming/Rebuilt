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
  private DoublePublisher shooterPositionLeftTop;
  private DoublePublisher shooterVelocityLeftTop;
  private DoublePublisher shooterAppliedVoltsLeftTop;
  private DoublePublisher shooterSupplyCurrentLeftTop;
  private DoublePublisher shooterTorqueCurrentLeftTop;
  private DoublePublisher shooterTempLeftTop;

  private DoublePublisher shooterPositionLeftBottom;
  private DoublePublisher shooterVelocityLeftBottom;
  private DoublePublisher shooterAppliedVoltsLeftBottom;
  private DoublePublisher shooterSupplyCurrentLeftBottom;
  private DoublePublisher shooterTorqueCurrentLeftBottom;
  private DoublePublisher shooterTempLeftBottom;

  private DoublePublisher shooterPositionRightTop;
  private DoublePublisher shooterVelocityRightTop;
  private DoublePublisher shooterAppliedVoltsRightTop;
  private DoublePublisher shooterSupplyCurrentRightTop;
  private DoublePublisher shooterTorqueCurrentRightTop;
  private DoublePublisher shooterTempRightTop;

  private DoublePublisher shooterPositionRightBottom;
  private DoublePublisher shooterVelocityRightBottom;
  private DoublePublisher shooterAppliedVoltsRightBottom;
  private DoublePublisher shooterSupplyCurrentRightBottom;
  private DoublePublisher shooterTorqueCurrentRightBottom;
  private DoublePublisher shooterTempRightBottom;

  private DoublePublisher hoodPosition;
  private DoublePublisher hoodVelocity;
  private DoublePublisher hoodAppliedVolts;
  private DoublePublisher hoodSupplyCurrent;
  private DoublePublisher hoodTorqueCurrent;
  private DoublePublisher hoodTemp;

  /* Initializes the shooter stats */
  class shooterStats {
    public boolean MotorConnected = true;    
    public double shooterPositionRadsLeftTop = 0.0;
    public double shooterVelocityRpmLeftTop = 0.0;
    public double shooterAppliedVoltsLeftTop = 0.0;
    public double shooterSupplyCurrentAmpsLeftTop = 0.0;
    public double shooterTorqueCurrentAmpsLeftTop = 0.0;
    public double shooterTempCelsiusLeftTop = 0.0;

    public boolean MotorConnectedLeftBottom = true;    
    public double shooterPositionRadsLeftBottom = 0.0;
    public double shooterVelocityRpmLeftBottom = 0.0;
    public double shooterAppliedVoltsLeftBottom = 0.0;
    public double shooterSupplyCurrentAmpsLeftBottom = 0.0;
    public double shooterTorqueCurrentAmpsLeftBottom = 0.0;
    public double shooterTempCelsiusLeftBottom = 0.0;

    public boolean MotorConnectedRightTop = true;    
    public double shooterPositionRadsRightTop = 0.0;
    public double shooterVelocityRpmRightTop = 0.0;
    public double shooterAppliedVoltsRightTop = 0.0;
    public double shooterSupplyCurrentAmpsRightTop = 0.0;
    public double shooterTorqueCurrentAmpsRightTop = 0.0;
    public double shooterTempCelsiusRightTop = 0.0;

    public boolean MotorConnectedRightBottom = true;    
    public double shooterPositionRadsRightBottom = 0.0;
    public double shooterVelocityRpmRightBottom = 0.0;
    public double shooterAppliedVoltsRightBottom = 0.0;
    public double shooterSupplyCurrentAmpsRightBottom = 0.0;
    public double shooterTorqueCurrentAmpsRightBottom = 0.0;
    public double shooterTempCelsiusRightBottom = 0.0;

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

    NetworkTable tableLeftTop = instance.getTable("HOT/Shooter/Shooter/Left Top (Master)");
    
    /* Creates the Display for the Network Tables */
    shooterPositionLeftTop = tableLeftTop.getDoubleTopic("Shooter Left Top (Master): Position-Motor (ROT)").publish();
    shooterVelocityLeftTop = tableLeftTop.getDoubleTopic("Shooter Left Top (Master): Velocity (RPS)").publish();
    shooterAppliedVoltsLeftTop = tableLeftTop.getDoubleTopic("Shooter Left Top (Master): Applied Volts").publish();
    shooterSupplyCurrentLeftTop = tableLeftTop.getDoubleTopic("Shooter Left Top (Master): Supply Current").publish();
    shooterTorqueCurrentLeftTop = tableLeftTop.getDoubleTopic("Shooter Left Top (Master): Torque Current").publish();
    shooterTempLeftTop = tableLeftTop.getDoubleTopic("Shooter Left Top (Master): Motor Temp").publish();

    NetworkTable tableLeftBottom = instance.getTable("HOT/Shooter/Shooter/Follower/Left Bottom");

    shooterPositionLeftBottom = tableLeftBottom.getDoubleTopic("Shooter Left Bottom: Position-Motor (ROT)").publish();
    shooterVelocityLeftBottom = tableLeftBottom.getDoubleTopic("Shooter Left Bottom: Velocity (RPS)").publish();
    shooterAppliedVoltsLeftBottom = tableLeftBottom.getDoubleTopic("Shooter Left Bottom: Applied Volts").publish();
    shooterSupplyCurrentLeftBottom = tableLeftBottom.getDoubleTopic("Shooter Left Bottom: Supply Current").publish();
    shooterTorqueCurrentLeftBottom = tableLeftBottom.getDoubleTopic("Shooter Left Bottom: Torque Current").publish();
    shooterTempLeftBottom = tableLeftBottom.getDoubleTopic("Shooter Left Bottom: Motor Temp").publish();

    NetworkTable tableRightTop = instance.getTable("HOT/Shooter/Shooter/Follower/Right Top");

    shooterPositionRightTop = tableRightTop.getDoubleTopic("Shooter Right Top: Position-Motor (ROT)").publish();
    shooterVelocityRightTop = tableRightTop.getDoubleTopic("Shooter Right Top: Velocity (RPS)").publish();
    shooterAppliedVoltsRightTop = tableRightTop.getDoubleTopic("Shooter Right Top: Applied Volts").publish();
    shooterSupplyCurrentRightTop = tableRightTop.getDoubleTopic("Shooter Right Top: Supply Current").publish();
    shooterTorqueCurrentRightTop = tableRightTop.getDoubleTopic("Shooter Right Top: Torque Current").publish();
    shooterTempRightTop = tableRightTop.getDoubleTopic("Shooter Right Top: Motor Temp").publish();

    NetworkTable tableRightBottom = instance.getTable("HOT/Shooter/Shooter/Follower/Right Bottom");

    shooterPositionRightBottom = tableRightBottom.getDoubleTopic("Shooter Right Bottom: Position-Motor (ROT)").publish();
    shooterVelocityRightBottom = tableRightBottom.getDoubleTopic("Shooter Right Bottom: Velocity (RPS)").publish();
    shooterAppliedVoltsRightBottom = tableRightBottom.getDoubleTopic("Shooter Right Bottom: Applied Volts").publish();
    shooterSupplyCurrentRightBottom = tableRightBottom.getDoubleTopic("Shooter Right Bottom: Supply Current").publish();
    shooterTorqueCurrentRightBottom = tableRightBottom.getDoubleTopic("Shooter Right Bottom: Torque Current").publish();
    shooterTempRightBottom = tableRightBottom.getDoubleTopic("Shooter Right Bottom: Motor Temp").publish();

    NetworkTable tableHood = instance.getTable("HOT/Shooter/Hood");

    hoodPosition = tableHood.getDoubleTopic("Hood: Position-Motor (ROT)").publish();
    hoodVelocity = tableHood.getDoubleTopic("Hood: Velocity (RPS)").publish();
    hoodAppliedVolts = tableHood.getDoubleTopic("Hood: Applied Volts").publish();
    hoodSupplyCurrent = tableHood.getDoubleTopic("Hood: Supply Current").publish();
    hoodTorqueCurrent = tableHood.getDoubleTopic("Hood: Torque Current").publish();
    hoodTemp = tableHood.getDoubleTopic("Hood: Motor Temp").publish();
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
    return motorStats.shooterVelocityRpmLeftTop;
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
    shooterVelocityLeftTop.set(motorStats.shooterVelocityRpmLeftTop);
    shooterPositionLeftTop.set(motorStats.shooterPositionRadsLeftTop);
    shooterSupplyCurrentLeftTop.set(motorStats.shooterSupplyCurrentAmpsLeftTop);
    shooterTorqueCurrentLeftTop.set(motorStats.shooterTorqueCurrentAmpsLeftTop);
    shooterAppliedVoltsLeftTop.set(motorStats.shooterAppliedVoltsLeftTop);
    shooterTempLeftTop.set(motorStats.shooterTempCelsiusLeftTop);

    shooterVelocityLeftBottom.set(motorStats.shooterVelocityRpmLeftBottom);
    shooterPositionLeftBottom.set(motorStats.shooterPositionRadsLeftBottom);
    shooterSupplyCurrentLeftBottom.set(motorStats.shooterSupplyCurrentAmpsLeftBottom);
    shooterTorqueCurrentLeftBottom.set(motorStats.shooterTorqueCurrentAmpsLeftBottom);
    shooterAppliedVoltsLeftBottom.set(motorStats.shooterAppliedVoltsLeftBottom);
    shooterTempLeftBottom.set(motorStats.shooterTempCelsiusLeftBottom);

    shooterVelocityRightTop.set(motorStats.shooterVelocityRpmRightTop);
    shooterPositionRightTop.set(motorStats.shooterPositionRadsRightTop);
    shooterSupplyCurrentRightTop.set(motorStats.shooterSupplyCurrentAmpsRightTop);
    shooterTorqueCurrentRightTop.set(motorStats.shooterTorqueCurrentAmpsRightTop);
    shooterAppliedVoltsRightTop.set(motorStats.shooterAppliedVoltsRightTop);
    shooterTempRightTop.set(motorStats.shooterTempCelsiusRightTop);

    shooterVelocityRightBottom.set(motorStats.shooterVelocityRpmRightBottom);
    shooterPositionRightBottom.set(motorStats.shooterPositionRadsRightBottom);
    shooterSupplyCurrentRightBottom.set(motorStats.shooterSupplyCurrentAmpsRightBottom);
    shooterTorqueCurrentRightBottom.set(motorStats.shooterTorqueCurrentAmpsRightBottom);
    shooterAppliedVoltsRightBottom.set(motorStats.shooterAppliedVoltsRightBottom);
    shooterTempRightBottom.set(motorStats.shooterTempCelsiusRightBottom);

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
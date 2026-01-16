package frc.robot.subsystems.Shooter;

import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.StringPublisher;
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
  private StringPublisher shooterState;

  class shooterStats {
    public boolean MotorConnected = true;    
    public double shooterPositionRads = 0.0;
    public double shooterVelocityRpm = 0.0;
    public double shooterAppliedVolts = 0.0;
    public double shooterSupplyCurrentAmps = 0.0;
    public double shooterTorqueCurrentAmps = 0.0;
    public double shooterTempCelsius = 0.0;  
  }

//Constructor for Shooter Object
    public Shooter() {
    }

    private void setShooterSpeed(int shooterSpeed) {
      mShooterKraken.runVelocity(shooterSpeed);
    }

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
    //shooterState.set(currentState.name());

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
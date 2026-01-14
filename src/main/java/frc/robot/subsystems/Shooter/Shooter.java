package frc.robot.subsystems.Shooter;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Shooter extends SubsystemBase {
    //Set and get velocity
    public Shooter() {
        
    }

    private void setShooterSpeed(int shooterSpeed) {

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

  public Command goToShoot() {
    return runOnce(
        () -> {
          /* one-time action goes here */
          setShooterSpeed(ShooterConstants.kshootSpeed);
        });
    }
}

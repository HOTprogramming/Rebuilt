package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.Shooter.Shooter.shooterStats;

public class ShooterKraken {
  private static ShooterKraken instance = null;
  private boolean isSimulation = false; 

   /**************************************************************** 
    * Change this variable to select which motor control feature runs 
    *     0-means run Velocity Voltage Control Mode  
    *     1-means run Velocity Torque Control Mode  
    *****************************************************************/           
  private int controlMode = 0; 
  
  // Initialize Motors //
  private TalonFX m_talonFX = new TalonFX(ShooterConstants.kMotorID, ShooterConstants.canbus);
  private TalonFX m_talonFXOther = new TalonFX(ShooterConstants.kOtherMotorID, ShooterConstants.canbus);

  // Initialize Control Modes //
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);    /* Start at velocity 0, use slot 0 */
  private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);  /* Start at velocity 0, use slot 1 */

  private final NeutralOut m_brake = new NeutralOut();   /* Keep a neutral out so we can disable the motor */
 
  // Status Signals //
  private final StatusSignal<Angle> shooterPosition;
  private final StatusSignal<AngularVelocity> shooterVelocity;
  private final StatusSignal<Voltage> shooterAppliedVolts;
  private final StatusSignal<Current> shooterSupplyCurrent;
  private final StatusSignal<Current> shooterTorqueCurrent;
  private final StatusSignal<Temperature> shooterTempCelsius;

  private final StatusSignal<Angle> shooterPositionOther;
  private final StatusSignal<AngularVelocity> shooterVelocityOther;
  private final StatusSignal<Voltage> shooterAppliedVoltsOther;
  private final StatusSignal<Current> shooterSupplyCurrentOther;
  private final StatusSignal<Current> shooterTorqueCurrentOther;
  private final StatusSignal<Temperature> shooterTempCelsiusOther;

  public ShooterKraken() {

    /**********************************
    * Confiure the Shooter Motor 
    ***********************************/
    // Apply Configuration from Shooter Constants to motor //
    StatusCode statuscode = m_talonFX.getConfigurator().apply(ShooterConstants.ShooterConfigs.shooterMotorConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Shooter Motor 1");
    }
    StatusCode statuscode2 = m_talonFXOther.getConfigurator().apply(ShooterConstants.ShooterConfigs.shooterMotorConfigs(), 1.0); 
    if(!statuscode2.isOK()) { 
      System.out.println("Configuration Failed on Shooter Motor 2");
    }

    // Initialize Telemetry data //
    shooterPosition = m_talonFX.getPosition();
    shooterVelocity = m_talonFX.getVelocity();
    shooterAppliedVolts = m_talonFX.getMotorVoltage();
    shooterSupplyCurrent = m_talonFX.getSupplyCurrent();
    shooterTorqueCurrent = m_talonFX.getTorqueCurrent();
    shooterTempCelsius = m_talonFX.getDeviceTemp();

    shooterPositionOther = m_talonFXOther.getPosition();
    shooterVelocityOther = m_talonFXOther.getVelocity();
    shooterAppliedVoltsOther = m_talonFXOther.getMotorVoltage();
    shooterSupplyCurrentOther = m_talonFXOther.getSupplyCurrent();
    shooterTorqueCurrentOther = m_talonFXOther.getTorqueCurrent();
    shooterTempCelsiusOther = m_talonFXOther.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        shooterPosition,
        shooterVelocity,
        shooterAppliedVolts,
        shooterSupplyCurrent,
        shooterTorqueCurrent,
        shooterTempCelsius
    );

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        shooterPositionOther,
        shooterVelocityOther,
        shooterAppliedVoltsOther,
        shooterSupplyCurrentOther,
        shooterTorqueCurrentOther,
        shooterTempCelsiusOther
    );

    /* Optimize out the other signals, since they're not useful */
    m_talonFX.optimizeBusUtilization();
    m_talonFXOther.optimizeBusUtilization();

    /* Defines the motor to the Physics Simulation-only in Simulation */
    if(Utils.isSimulation()){
      isSimulation = true; 
      PhysicsSim.getInstance().addTalonFX(m_talonFX, 0.001);
      PhysicsSim.getInstance().addTalonFX(m_talonFXOther, 0.001);
    }
  }

  /*****************************************************************************
   * Return an instance of this class - Ensures that there is a single instance   
   ****************************************************************************/
  public static ShooterKraken getInstance() {
    return instance = instance == null ? new ShooterKraken() : instance;
  }

    public void updateStats(shooterStats stats) {
      if(isSimulation) {
        PhysicsSim.getInstance().run();
      }

      stats.MotorConnected =
        BaseStatusSignal.refreshAll(
          shooterPosition,
          shooterVelocity,
          shooterAppliedVolts,
          shooterSupplyCurrent,
          shooterTorqueCurrent,
          shooterTempCelsius,
          
          shooterPositionOther,
          shooterVelocityOther,
          shooterAppliedVoltsOther,
          shooterSupplyCurrentOther,
          shooterTorqueCurrentOther,
          shooterTempCelsiusOther
          )
        .isOK();
      
      // Set telemetry data to Status Signals //
      stats.shooterPositionRads = shooterPosition.getValueAsDouble();
      stats.shooterVelocityRpm = shooterVelocity.getValueAsDouble();
      stats.shooterAppliedVolts = shooterAppliedVolts.getValueAsDouble();
      stats.shooterSupplyCurrentAmps = shooterSupplyCurrent.getValueAsDouble();
      stats.shooterTorqueCurrentAmps = shooterTorqueCurrent.getValueAsDouble();
      stats.shooterTempCelsius = shooterTempCelsius.getValueAsDouble();

      stats.shooterPositionRadsOther = shooterPositionOther.getValueAsDouble();
      stats.shooterVelocityRpmOther = shooterVelocityOther.getValueAsDouble();
      stats.shooterAppliedVoltsOther = shooterAppliedVoltsOther.getValueAsDouble();
      stats.shooterSupplyCurrentAmpsOther = shooterSupplyCurrentOther.getValueAsDouble();
      stats.shooterTorqueCurrentAmpsOther = shooterTorqueCurrentOther.getValueAsDouble();
      stats.shooterTempCelsiusOther = shooterTempCelsiusOther.getValueAsDouble();
    }

    /*********************
  * The runVelocity function can run (based on the controlMode Variable either:
  *     0-means run Velocity Voltage Control Mode  
  *     1-means run Velocity Torque Control Mode  
  **********************/
  public void runVelocity(double RPM) {      
    //Convert rotations per minute into rotations per second 
    double desiredRotationsPerSecond = (RPM/60.0); 

    switch (controlMode) {
      case 0:      /* Use velocity voltage */
        m_talonFX.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
        m_talonFXOther.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
      case 1:      /* Use velocity torque */
        m_talonFX.setControl(m_velocityTorque.withVelocity(desiredRotationsPerSecond));
        m_talonFXOther.setControl(m_velocityTorque.withVelocity(desiredRotationsPerSecond));
      }
  }

  public void stop() {
    /* tell the motor to brake */
    m_talonFX.setControl(m_brake);
    m_talonFXOther.setControl(m_brake);
  }
}
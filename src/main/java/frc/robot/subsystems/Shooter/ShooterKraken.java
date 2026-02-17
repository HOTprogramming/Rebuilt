package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.MotorAlignmentValue;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.Shooter.Shooter.shooterStats;

public class ShooterKraken {
  private static ShooterKraken instance = null;

  /**************************************************************** 
  * Change this variable to select which motor control feature runs
  * Control Mode Shooter:
  *     0-means run Velocity Voltage Control Mode  
  *     1-means run Velocity Torque Control Mode  
  * Control Mode Hood:
  *     0-means run Position Voltage Control Mode  
  *     1-means run Position Torque Control Mode
  *****************************************************************/           
  private int controlModeShoot = 1; 
  private int controlModeHood = 0;
  
  // Initialize Motors //
  private TalonFX m_talonFXLeftTop = new TalonFX(ShooterConstants.kMotorIDLeftTop, ShooterConstants.canbus);
  private TalonFX m_talonFXLeftBottom = new TalonFX(ShooterConstants.kMotorIDLeftBottom, ShooterConstants.canbus);
  private TalonFX m_talonFXRightTop = new TalonFX(ShooterConstants.kMotorIDRightTop, ShooterConstants.canbus);
  private TalonFX m_talonFXRightBottom = new TalonFX(ShooterConstants.kMotorIDRightBottom, ShooterConstants.canbus);
  private TalonFX m_talonFXHood = new TalonFX(ShooterConstants.kHoodMotorID, ShooterConstants.canbus);

  // Initialize Control Modes //
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);    /* Start at velocity 0, use slot 0 */
  private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);  /* Start at velocity 0, use slot 1 */

  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);

  private final NeutralOut m_brake = new NeutralOut();   /* Keep a neutral out so we can disable the motor */

  StatusCode statuscode;
 
  // Status Signals //
  private final StatusSignal<Angle> shooterPositionLeftTop;
  private final StatusSignal<AngularVelocity> shooterVelocityLeftTop;
  private final StatusSignal<Voltage> shooterAppliedVoltsLeftTop;
  private final StatusSignal<Current> shooterSupplyCurrentLeftTop;
  private final StatusSignal<Current> shooterTorqueCurrentLeftTop;
  private final StatusSignal<Temperature> shooterTempCelsiusLeftTop;

  private final StatusSignal<Angle> shooterPositionLeftBottom;
  private final StatusSignal<AngularVelocity> shooterVelocityLeftBottom;
  private final StatusSignal<Voltage> shooterAppliedVoltsLeftBottom;
  private final StatusSignal<Current> shooterSupplyCurrentLeftBottom;
  private final StatusSignal<Current> shooterTorqueCurrentLeftBottom;
  private final StatusSignal<Temperature> shooterTempCelsiusLeftBottom;

  private final StatusSignal<Angle> shooterPositionRightTop;
  private final StatusSignal<AngularVelocity> shooterVelocityRightTop;
  private final StatusSignal<Voltage> shooterAppliedVoltsRightTop;
  private final StatusSignal<Current> shooterSupplyCurrentRightTop;
  private final StatusSignal<Current> shooterTorqueCurrentRightTop;
  private final StatusSignal<Temperature> shooterTempCelsiusRightTop;

  private final StatusSignal<Angle> shooterPositionRightBottom;
  private final StatusSignal<AngularVelocity> shooterVelocityRightBottom;
  private final StatusSignal<Voltage> shooterAppliedVoltsRightBottom;
  private final StatusSignal<Current> shooterSupplyCurrentRightBottom;
  private final StatusSignal<Current> shooterTorqueCurrentRightBottom;
  private final StatusSignal<Temperature> shooterTempCelsiusRightBottom;

  private final StatusSignal<Angle> hoodPosition;
  private final StatusSignal<AngularVelocity> hoodVelocity;
  private final StatusSignal<Voltage> hoodAppliedVolts;
  private final StatusSignal<Current> hoodSupplyCurrent;
  private final StatusSignal<Current> hoodTorqueCurrent;
  private final StatusSignal<Temperature> hoodTempCelsius;

  public ShooterKraken() {

    /**********************************
    * Confiure the Shooter Motor 
    ***********************************/
    // Apply Configuration from Shooter Constants to motor //
    statuscode = m_talonFXLeftTop.getConfigurator().apply(ShooterConstants.ShooterConfigs.shooterMotorConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Shooter Left Top (Master) Motor");
    }
    statuscode = m_talonFXLeftBottom.getConfigurator().apply(ShooterConstants.ShooterConfigs.shooterMotorConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Shooter Left Bottom Motor");
    }
    statuscode = m_talonFXRightTop.getConfigurator().apply(ShooterConstants.ShooterConfigs.shooterMotorConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Shooter Right Top Motor");
    }
    statuscode = m_talonFXRightBottom.getConfigurator().apply(ShooterConstants.ShooterConfigs.shooterMotorConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Shooter Right Bottom Motor");
    }
    statuscode = m_talonFXHood.getConfigurator().apply(ShooterConstants.ShooterConfigs.hoodMotorConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Hood Motor");
    }

    m_talonFXLeftBottom.setControl(new Follower(ShooterConstants.kMotorIDLeftTop, MotorAlignmentValue.Opposed));
    m_talonFXRightTop.setControl(new Follower(ShooterConstants.kMotorIDLeftTop, MotorAlignmentValue.Opposed));
    m_talonFXRightBottom.setControl(new Follower(ShooterConstants.kMotorIDLeftTop, MotorAlignmentValue.Opposed));

    // Initialize Telemetry data //
    shooterPositionLeftTop = m_talonFXLeftTop.getPosition();
    shooterVelocityLeftTop = m_talonFXLeftTop.getVelocity();
    shooterAppliedVoltsLeftTop = m_talonFXLeftTop.getMotorVoltage();
    shooterSupplyCurrentLeftTop = m_talonFXLeftTop.getSupplyCurrent();
    shooterTorqueCurrentLeftTop = m_talonFXLeftTop.getTorqueCurrent();
    shooterTempCelsiusLeftTop = m_talonFXLeftTop.getDeviceTemp();

    shooterPositionLeftBottom = m_talonFXLeftBottom.getPosition();
    shooterVelocityLeftBottom = m_talonFXLeftBottom.getVelocity();
    shooterAppliedVoltsLeftBottom = m_talonFXLeftBottom.getMotorVoltage();
    shooterSupplyCurrentLeftBottom = m_talonFXLeftBottom.getSupplyCurrent();
    shooterTorqueCurrentLeftBottom = m_talonFXLeftBottom.getTorqueCurrent();
    shooterTempCelsiusLeftBottom = m_talonFXLeftBottom.getDeviceTemp();

    shooterPositionRightTop = m_talonFXRightTop.getPosition();
    shooterVelocityRightTop = m_talonFXRightTop.getVelocity();
    shooterAppliedVoltsRightTop = m_talonFXRightTop.getMotorVoltage();
    shooterSupplyCurrentRightTop = m_talonFXRightTop.getSupplyCurrent();
    shooterTorqueCurrentRightTop = m_talonFXRightTop.getTorqueCurrent();
    shooterTempCelsiusRightTop = m_talonFXRightTop.getDeviceTemp();

    shooterPositionRightBottom = m_talonFXRightBottom.getPosition();
    shooterVelocityRightBottom = m_talonFXRightBottom.getVelocity();
    shooterAppliedVoltsRightBottom = m_talonFXRightBottom.getMotorVoltage();
    shooterSupplyCurrentRightBottom = m_talonFXRightBottom.getSupplyCurrent();
    shooterTorqueCurrentRightBottom = m_talonFXRightBottom.getTorqueCurrent();
    shooterTempCelsiusRightBottom = m_talonFXRightBottom.getDeviceTemp();

    hoodPosition = m_talonFXHood.getPosition();
    hoodVelocity = m_talonFXHood.getVelocity();
    hoodAppliedVolts = m_talonFXHood.getMotorVoltage();
    hoodSupplyCurrent = m_talonFXHood.getSupplyCurrent();
    hoodTorqueCurrent = m_talonFXHood.getTorqueCurrent();
    hoodTempCelsius = m_talonFXHood.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
      100.0,
      shooterPositionLeftTop,
      shooterVelocityLeftTop,
      shooterAppliedVoltsLeftTop,
      shooterSupplyCurrentLeftTop,
      shooterTorqueCurrentLeftTop,
      shooterTempCelsiusLeftTop,

      shooterPositionLeftBottom,
      shooterVelocityLeftBottom,
      shooterAppliedVoltsLeftBottom,
      shooterSupplyCurrentLeftBottom,
      shooterTorqueCurrentLeftBottom,
      shooterTempCelsiusLeftBottom,

      shooterPositionRightTop,
      shooterVelocityRightTop,
      shooterAppliedVoltsRightTop,
      shooterSupplyCurrentRightTop,
      shooterTorqueCurrentRightTop,
      shooterTempCelsiusRightTop,

      shooterPositionRightBottom,
      shooterVelocityRightBottom,
      shooterAppliedVoltsRightBottom,
      shooterSupplyCurrentRightBottom,
      shooterTorqueCurrentRightBottom,
      shooterTempCelsiusRightBottom,

      hoodPosition,
      hoodVelocity,
      hoodAppliedVolts,
      hoodSupplyCurrent,
      hoodTorqueCurrent,
      hoodTempCelsius
    );

    /* Optimize out the other signals, since they're not useful */
    m_talonFXLeftTop.optimizeBusUtilization();
    m_talonFXLeftBottom.optimizeBusUtilization();
    m_talonFXRightTop.optimizeBusUtilization();
    m_talonFXRightBottom.optimizeBusUtilization();
    m_talonFXHood.optimizeBusUtilization();

    /* Defines the motor to the Physics Simulation-only in Simulation */
    if(Utils.isSimulation()){
      PhysicsSim.getInstance().addTalonFX(m_talonFXLeftTop, 0.001);
      PhysicsSim.getInstance().addTalonFX(m_talonFXLeftBottom, 0.001);
      PhysicsSim.getInstance().addTalonFX(m_talonFXRightTop, 0.001);
      PhysicsSim.getInstance().addTalonFX(m_talonFXRightBottom, 0.001);
      PhysicsSim.getInstance().addTalonFX(m_talonFXHood, 0.001);
    }
  }

  /*****************************************************************************
   * Return an instance of this class - Ensures that there is a single instance   
   ****************************************************************************/
  public static ShooterKraken getInstance() {
    return instance = instance == null ? new ShooterKraken() : instance;
  }

  public void updateStats(shooterStats stats) {
    if(Utils.isSimulation()) {
      PhysicsSim.getInstance().run();
    }

    stats.MotorConnected =BaseStatusSignal.refreshAll(
      shooterPositionLeftTop,
      shooterVelocityLeftTop,
      shooterAppliedVoltsLeftTop,
      shooterSupplyCurrentLeftTop,
      shooterTorqueCurrentLeftTop,
      shooterTempCelsiusLeftTop,
      
      shooterPositionLeftBottom,
      shooterVelocityLeftBottom,
      shooterAppliedVoltsLeftBottom,
      shooterSupplyCurrentLeftBottom,
      shooterTorqueCurrentLeftBottom,
      shooterTempCelsiusLeftBottom,

      shooterPositionRightTop,
      shooterVelocityRightTop,
      shooterAppliedVoltsRightTop,
      shooterSupplyCurrentRightTop,
      shooterTorqueCurrentRightTop,
      shooterTempCelsiusRightTop,

      shooterPositionRightBottom,
      shooterVelocityRightBottom,
      shooterAppliedVoltsRightBottom,
      shooterSupplyCurrentRightBottom,
      shooterTorqueCurrentRightBottom,
      shooterTempCelsiusRightBottom,

      hoodPosition,
      hoodVelocity,
      hoodAppliedVolts,
      hoodSupplyCurrent,
      hoodTorqueCurrent,
      hoodTempCelsius
    ).isOK();
      
    // Set telemetry data to Status Signals //
    stats.shooterPositionRadsLeftTop = shooterPositionLeftTop.getValueAsDouble();
    stats.shooterVelocityRpmLeftTop = shooterVelocityLeftTop.getValueAsDouble();
    stats.shooterAppliedVoltsLeftTop = shooterAppliedVoltsLeftTop.getValueAsDouble();
    stats.shooterSupplyCurrentAmpsLeftTop = shooterSupplyCurrentLeftTop.getValueAsDouble();
    stats.shooterTorqueCurrentAmpsLeftTop = shooterTorqueCurrentLeftTop.getValueAsDouble();
    stats.shooterTempCelsiusLeftTop = shooterTempCelsiusLeftTop.getValueAsDouble();

    stats.shooterPositionRadsLeftBottom = shooterPositionLeftBottom.getValueAsDouble();
    stats.shooterVelocityRpmLeftBottom = shooterVelocityLeftBottom.getValueAsDouble();
    stats.shooterAppliedVoltsLeftBottom = shooterAppliedVoltsLeftBottom.getValueAsDouble();
    stats.shooterSupplyCurrentAmpsLeftBottom = shooterSupplyCurrentLeftBottom.getValueAsDouble();
    stats.shooterTorqueCurrentAmpsLeftBottom = shooterTorqueCurrentLeftBottom.getValueAsDouble();
    stats.shooterTempCelsiusLeftBottom = shooterTempCelsiusLeftBottom.getValueAsDouble();

    stats.shooterPositionRadsRightTop = shooterPositionRightTop.getValueAsDouble();
    stats.shooterVelocityRpmRightTop = shooterVelocityRightTop.getValueAsDouble();
    stats.shooterAppliedVoltsRightTop = shooterAppliedVoltsRightTop.getValueAsDouble();
    stats.shooterSupplyCurrentAmpsRightTop = shooterSupplyCurrentRightTop.getValueAsDouble();
    stats.shooterTorqueCurrentAmpsRightTop = shooterTorqueCurrentRightTop.getValueAsDouble();
    stats.shooterTempCelsiusRightTop = shooterTempCelsiusRightTop.getValueAsDouble();


    stats.shooterPositionRadsRightBottom = shooterPositionRightBottom.getValueAsDouble();
    stats.shooterVelocityRpmRightBottom = shooterVelocityRightBottom.getValueAsDouble();
    stats.shooterAppliedVoltsRightBottom = shooterAppliedVoltsRightBottom.getValueAsDouble();
    stats.shooterSupplyCurrentAmpsRightBottom = shooterSupplyCurrentRightBottom.getValueAsDouble();
    stats.shooterTorqueCurrentAmpsRightBottom = shooterTorqueCurrentRightBottom.getValueAsDouble();
    stats.shooterTempCelsiusRightBottom = shooterTempCelsiusRightBottom.getValueAsDouble();

    stats.hoodPositionRads = hoodPosition.getValueAsDouble();
    stats.hoodVelocityRpm = hoodVelocity.getValueAsDouble();
    stats.hoodAppliedVolts = hoodAppliedVolts.getValueAsDouble();
    stats.hoodSupplyCurrentAmps = hoodSupplyCurrent.getValueAsDouble();
    stats.hoodTorqueCurrentAmps = hoodTorqueCurrent.getValueAsDouble();
    stats.hoodTempCelsius = hoodTempCelsius.getValueAsDouble();
  }

  /*********************
  * The runVelocity function can run based on the controlMode Variable either:
  *     0-means run Velocity Voltage Control Mode  
  *     1-means run Velocity Torque Control Mode  
  **********************/
  public void runVelocity(double desiredRotationsPerSecond) {      
    switch (controlModeShoot) {
      case 0:      /* Use velocity voltage */
        m_talonFXLeftTop.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
        break;
      case 1:      /* Use velocity torque */
        m_talonFXLeftTop.setControl(m_velocityTorque.withVelocity(desiredRotationsPerSecond));
    }
  }

  /***************************************************************************
  * The runVelocity function can run (based on the controlMode Variable either:
  *     0-means run Position Voltage Control Mode  
  *     1-means run Position Torque Control Mode  
  ***************************************************************************/
  public void runPosition(double desiredRotations) {      
      switch (controlModeHood) {
        case 0:      /* Use Position Voltage */
            m_talonFXHood.setControl(m_positionVoltage.withPosition(desiredRotations));
            break;
        case 1:      /* Use Position Torque */
            m_talonFXHood.setControl(m_positionTorque.withPosition(desiredRotations));
        }
    }

  public void stop() {
    /* tell the motor to brake */
    m_talonFXLeftTop.setControl(m_brake);
  }
}
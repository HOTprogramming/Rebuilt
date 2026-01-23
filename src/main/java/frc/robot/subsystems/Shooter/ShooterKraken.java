package frc.robot.subsystems.Shooter;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.NeutralOut;
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
  private boolean isSimulation = false; 

   /**************************************************************** 
    * Change this variable to select which motor control feature runs 
    *     0-means run Velocity Voltage Control Mode  
    *     1-means run Velocity Torque Control Mode  
    *****************************************************************/           
  private int controlMode = 1; 
  
  // Initialize Motors //
  private TalonFX m_talonFX = new TalonFX(ShooterConstants.kMotorID, ShooterConstants.canbus);
  private TalonFX m_talonFXSub = new TalonFX(ShooterConstants.kSubMotorID, ShooterConstants.canbus);
  private TalonFX m_talonFXSubSub = new TalonFX(ShooterConstants.kSubSubMotorID, ShooterConstants.canbus);

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

  private final StatusSignal<Angle> shooterPositionSub;
  private final StatusSignal<AngularVelocity> shooterVelocitySub;
  private final StatusSignal<Voltage> shooterAppliedVoltsSub;
  private final StatusSignal<Current> shooterSupplyCurrentSub;
  private final StatusSignal<Current> shooterTorqueCurrentSub;
  private final StatusSignal<Temperature> shooterTempCelsiusSub;

  private final StatusSignal<Angle> shooterPositionSubSub;
  private final StatusSignal<AngularVelocity> shooterVelocitySubSub;
  private final StatusSignal<Voltage> shooterAppliedVoltsSubSub;
  private final StatusSignal<Current> shooterSupplyCurrentSubSub;
  private final StatusSignal<Current> shooterTorqueCurrentSubSub;
  private final StatusSignal<Temperature> shooterTempCelsiusSubSub;

  public ShooterKraken() {

    /**********************************
    * Confiure the Shooter Motor 
    ***********************************/
    // Apply Configuration from Shooter Constants to motor //
    StatusCode statuscode = m_talonFX.getConfigurator().apply(ShooterConstants.ShooterConfigs.shooterMasterConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Shooter Master Motor");
    }
    StatusCode statuscode2 = m_talonFXSub.getConfigurator().apply(ShooterConstants.ShooterConfigs.shooterFollowerConfigs(), 1.0); 
    if(!statuscode2.isOK()) { 
      System.out.println("Configuration Failed on Shooter Follower Motor");
    }
    StatusCode statuscode3 = m_talonFXSubSub.getConfigurator().apply(ShooterConstants.ShooterConfigs.shooterFollowerConfigs(), 1.0); 
    if(!statuscode3.isOK()) { 
      System.out.println("Configuration Failed on Shooter Follower Motor");
    }

    // Set-up 2 follower motors to follow master motor //
    m_talonFXSub.setControl(new Follower(m_talonFX.getDeviceID(),MotorAlignmentValue.Aligned));
    m_talonFXSubSub.setControl(new Follower(m_talonFX.getDeviceID(),MotorAlignmentValue.Aligned));

    // Initialize Telemetry data //
    shooterPosition = m_talonFX.getPosition();
    shooterVelocity = m_talonFX.getVelocity();
    shooterAppliedVolts = m_talonFX.getMotorVoltage();
    shooterSupplyCurrent = m_talonFX.getSupplyCurrent();
    shooterTorqueCurrent = m_talonFX.getTorqueCurrent();
    shooterTempCelsius = m_talonFX.getDeviceTemp();

    shooterPositionSub = m_talonFXSub.getPosition();
    shooterVelocitySub = m_talonFXSub.getVelocity();
    shooterAppliedVoltsSub = m_talonFXSub.getMotorVoltage();
    shooterSupplyCurrentSub = m_talonFXSub.getSupplyCurrent();
    shooterTorqueCurrentSub = m_talonFXSub.getTorqueCurrent();
    shooterTempCelsiusSub = m_talonFXSub.getDeviceTemp();

    shooterPositionSubSub = m_talonFXSubSub.getPosition();
    shooterVelocitySubSub = m_talonFXSubSub.getVelocity();
    shooterAppliedVoltsSubSub = m_talonFXSubSub.getMotorVoltage();
    shooterSupplyCurrentSubSub = m_talonFXSubSub.getSupplyCurrent();
    shooterTorqueCurrentSubSub = m_talonFXSubSub.getTorqueCurrent();
    shooterTempCelsiusSubSub = m_talonFXSubSub.getDeviceTemp();

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
        shooterPositionSub,
        shooterVelocitySub,
        shooterAppliedVoltsSub,
        shooterSupplyCurrentSub,
        shooterTorqueCurrentSub,
        shooterTempCelsiusSub
    );

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        shooterPositionSubSub,
        shooterVelocitySubSub,
        shooterAppliedVoltsSubSub,
        shooterSupplyCurrentSubSub,
        shooterTorqueCurrentSubSub,
        shooterTempCelsiusSubSub
    );

    /* Optimize out the other signals, since they're not useful */
    m_talonFX.optimizeBusUtilization();
    m_talonFXSub.optimizeBusUtilization();
    m_talonFXSubSub.optimizeBusUtilization();

    /* Defines the motor to the Physics Simulation-only in Simulation */
    if(Utils.isSimulation()){
      isSimulation = true; 
      PhysicsSim.getInstance().addTalonFX(m_talonFX, 0.001);
      PhysicsSim.getInstance().addTalonFX(m_talonFXSub, 0.001);
      PhysicsSim.getInstance().addTalonFX(m_talonFXSubSub, 0.001);
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
          
          shooterPositionSub,
          shooterVelocitySub,
          shooterAppliedVoltsSub,
          shooterSupplyCurrentSub,
          shooterTorqueCurrentSub,
          shooterTempCelsiusSub,
          
          shooterPositionSubSub,
          shooterVelocitySubSub,
          shooterAppliedVoltsSubSub,
          shooterSupplyCurrentSubSub,
          shooterTorqueCurrentSubSub,
          shooterTempCelsiusSubSub
          )
        .isOK();
      
      // Set telemetry data to Status Signals //
      stats.shooterPositionRads = shooterPosition.getValueAsDouble();
      stats.shooterVelocityRpm = shooterVelocity.getValueAsDouble();
      stats.shooterAppliedVolts = shooterAppliedVolts.getValueAsDouble();
      stats.shooterSupplyCurrentAmps = shooterSupplyCurrent.getValueAsDouble();
      stats.shooterTorqueCurrentAmps = shooterTorqueCurrent.getValueAsDouble();
      stats.shooterTempCelsius = shooterTempCelsius.getValueAsDouble();

      stats.shooterPositionRadsSub = shooterPositionSub.getValueAsDouble();
      stats.shooterVelocityRpmSub = shooterVelocitySub.getValueAsDouble();
      stats.shooterAppliedVoltsSub = shooterAppliedVoltsSub.getValueAsDouble();
      stats.shooterSupplyCurrentAmpsSub = shooterSupplyCurrentSub.getValueAsDouble();
      stats.shooterTorqueCurrentAmpsSub = shooterTorqueCurrentSub.getValueAsDouble();
      stats.shooterTempCelsiusSub = shooterTempCelsiusSub.getValueAsDouble();

      stats.shooterPositionRadsSubSub = shooterPositionSubSub.getValueAsDouble();
      stats.shooterVelocityRpmSubSub = shooterVelocitySubSub.getValueAsDouble();
      stats.shooterAppliedVoltsSubSub = shooterAppliedVoltsSubSub.getValueAsDouble();
      stats.shooterSupplyCurrentAmpsSubSub = shooterSupplyCurrentSubSub.getValueAsDouble();
      stats.shooterTorqueCurrentAmpsSubSub = shooterTorqueCurrentSubSub.getValueAsDouble();
      stats.shooterTempCelsiusSubSub = shooterTempCelsiusSubSub.getValueAsDouble();
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
      case 1:      /* Use velocity torque */
        m_talonFX.setControl(m_velocityTorque.withVelocity(desiredRotationsPerSecond));
      }
  }

  public void stop() {
    /* tell the motor to brake */
    m_talonFX.setControl(m_brake);
  }
}

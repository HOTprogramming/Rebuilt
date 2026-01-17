package frc.robot.subsystems.Intake;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.Intake.Intake.IntakeStats;

public class IntakeKraken {
  // Hardware
  private CANBus canBus = new CANBus(IntakeConstants.kBusName);
  private TalonFX m_LExtend = new TalonFX(IntakeConstants.kIntakeMotorIDLExtend, IntakeConstants.kBusName);
  private TalonFX m_RExtend = new TalonFX(IntakeConstants.kIntakeMotorIDRExtend, IntakeConstants.kBusName);
  private TalonFX m_LRoller = new TalonFX(IntakeConstants.kIntakeMotorIDLRoller, IntakeConstants.kBusName);
  private TalonFX m_RRoller = new TalonFX(IntakeConstants.kIntakeMotorIDRRoller, IntakeConstants.kBusName);
    //Control
  private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
  private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
  private final MotionMagicExpoVoltage m_mmExpoVoltage = new MotionMagicExpoVoltage(0).withSlot(2);
  private final MotionMagicExpoTorqueCurrentFOC m_mmExpoTorque = new MotionMagicExpoTorqueCurrentFOC(0.0).withSlot(1);
  private final VelocityVoltage m_velocityVoltage = new VelocityVoltage(0).withSlot(0);    /* Start at velocity 0, use slot 0 */
  private final VelocityTorqueCurrentFOC m_velocityTorque = new VelocityTorqueCurrentFOC(0).withSlot(1);  /* Start at velocity 0, use slot 1 */
  private final MotionMagicVoltage m_mmVelocityVoltage = new MotionMagicVoltage(0).withSlot(2);
  private final MotionMagicTorqueCurrentFOC m_mmVelocityTorque = new MotionMagicTorqueCurrentFOC(0.0).withSlot(1);
  private final NeutralOut m_brake = new NeutralOut();   /* Keep a neutral out so we can disable the motor */
  
  private final StatusSignal<Angle> LeftIntakePosition;
  private final StatusSignal<AngularVelocity> LeftIntakeVelocity;
  private final StatusSignal<Voltage> LeftIntakeAppliedVolts;
  private final StatusSignal<Current> LeftIntakeSupplyCurrent;
  private final StatusSignal<Current> LeftIntakeTorqueCurrent;
  private final StatusSignal<Temperature> LeftIntakeTempCelsius;

  private final StatusSignal<Angle> RightIntakePosition;
  private final StatusSignal<AngularVelocity> RightIntakeVelocity;
  private final StatusSignal<Voltage> RightIntakeAppliedVolts;
  private final StatusSignal<Current> RightIntakeSupplyCurrent;
  private final StatusSignal<Current> RightIntakeTorqueCurrent;
  private final StatusSignal<Temperature> RightIntakeTempCelsius;

  private final StatusSignal<Angle> LeftRollerPosition;
  private final StatusSignal<AngularVelocity> LeftRollerVelocity;
  private final StatusSignal<Voltage> LeftRollerAppliedVolts;
  private final StatusSignal<Current> LeftRollerSupplyCurrent;
  private final StatusSignal<Current> LeftRollerTorqueCurrent;
  private final StatusSignal<Temperature> LeftRollerTempCelsius;

  private final StatusSignal<Angle> RightRollerPosition;
  private final StatusSignal<AngularVelocity> RightRollerVelocity;
  private final StatusSignal<Voltage> RightRollerAppliedVolts;
  private final StatusSignal<Current> RightRollerSupplyCurrent;
  private final StatusSignal<Current> RightRollerTorqueCurrent;
  private final StatusSignal<Temperature> RightRollerTempCelsius;

  private static IntakeKraken instance = null;
  private double appliedVolts = 0.0; 
  private boolean isSimulation = false; 

   /**************************************************************** 
    * Change this variable to select which motor control feature runs 
    *     0-means run Velocity Voltage Control Mode  
    *     1-means run Velocity Torque Control Mode  
    *     2-means run Motion Magic Voltage Control Mode  
    *     3-means run Motion Magic Torque Control mode  
    *****************************************************************/           
  private int controlPos = 3; 
  private int controlVel = 3;
  private StatusCode statuscode = null;

  public IntakeKraken(){
         /**********************************
    * Confiure the shooter Motor 
    ***********************************/
    // Apply Configuration from Intake Constants to motor 
    statuscode = m_LExtend.getConfigurator().apply(IntakeConstants.IntakeConfigs.LeftIntakeExtendConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Intake shooter Motor ");
    } 
    statuscode = m_RExtend.getConfigurator().apply(IntakeConstants.IntakeConfigs.RightIntakeExtendConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Intake shooter Motor ");
    } 
    statuscode = m_LRoller.getConfigurator().apply(IntakeConstants.IntakeConfigs.LeftRollerConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Intake shooter Motor ");
    } 
    statuscode = m_RRoller.getConfigurator().apply(IntakeConstants.IntakeConfigs.RightRollerConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Intake shooter Motor ");
    } 

    // Set signals
    LeftIntakePosition = m_LExtend.getPosition();
    LeftIntakeVelocity = m_LExtend.getVelocity();
    LeftIntakeAppliedVolts = m_LExtend.getMotorVoltage();
    LeftIntakeSupplyCurrent = m_LExtend.getSupplyCurrent();
    LeftIntakeTorqueCurrent = m_LExtend.getTorqueCurrent();
    LeftIntakeTempCelsius = m_LExtend.getDeviceTemp();

    RightIntakePosition = m_RExtend.getPosition();
    RightIntakeVelocity = m_RExtend.getVelocity();
    RightIntakeAppliedVolts = m_RExtend.getMotorVoltage();
    RightIntakeSupplyCurrent = m_RExtend.getSupplyCurrent();
    RightIntakeTorqueCurrent = m_RExtend.getTorqueCurrent();
    RightIntakeTempCelsius = m_RExtend.getDeviceTemp();
  
    LeftRollerPosition = m_LRoller.getPosition();
    LeftRollerVelocity = m_LRoller.getVelocity();
    LeftRollerAppliedVolts = m_LRoller.getMotorVoltage();
    LeftRollerSupplyCurrent = m_LRoller.getSupplyCurrent();
    LeftRollerTorqueCurrent = m_LRoller.getTorqueCurrent();
    LeftRollerTempCelsius = m_LRoller.getDeviceTemp();

    RightRollerPosition = m_RRoller.getPosition();
    RightRollerVelocity = m_RRoller.getVelocity();
    RightRollerAppliedVolts = m_RRoller.getMotorVoltage();
    RightRollerSupplyCurrent = m_RRoller.getSupplyCurrent();
    RightRollerTorqueCurrent = m_RRoller.getTorqueCurrent();
    RightRollerTempCelsius = m_RRoller.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
          LeftIntakePosition,
        LeftIntakeVelocity,
        LeftIntakeAppliedVolts,
        LeftIntakeSupplyCurrent,
        LeftIntakeTorqueCurrent,
        LeftIntakeTempCelsius,
          RightIntakePosition,
        RightIntakeVelocity,
        RightIntakeAppliedVolts,
        RightIntakeSupplyCurrent,
        RightIntakeTorqueCurrent,
        RightIntakeTempCelsius,
          LeftRollerPosition,
        LeftRollerVelocity,
        LeftRollerAppliedVolts,
        LeftRollerSupplyCurrent,
        LeftRollerTorqueCurrent,
        LeftRollerTempCelsius,
          RightRollerPosition,
        RightRollerVelocity,
        RightRollerAppliedVolts,
        RightRollerSupplyCurrent,
        RightRollerTorqueCurrent,
        RightRollerTempCelsius
    );

    /* Optimize out the other signals, since they're not useful */
    m_LExtend.optimizeBusUtilization();
     m_RExtend.optimizeBusUtilization();
      m_LRoller.optimizeBusUtilization();
       m_RRoller.optimizeBusUtilization();

    /* Defines the motor to the Physics Simulation-only in Simulation */
    if(Utils.isSimulation()){
      isSimulation = true; 
      PhysicsSim.getInstance().addTalonFX(m_LExtend, 0.001);
      PhysicsSim.getInstance().addTalonFX(m_RExtend, 0.001);
      PhysicsSim.getInstance().addTalonFX(m_LRoller, 0.001);
      PhysicsSim.getInstance().addTalonFX(m_RRoller, 0.001);
      }
    }

  public static IntakeKraken getInstance() {
    return instance = instance == null ? new IntakeKraken() : instance;
  }
  public void updateStats(IntakeStats stats) {
  if(isSimulation){
      PhysicsSim.getInstance().run();
    }

    stats.MotorConnected =
        BaseStatusSignal.refreshAll(
                LeftIntakePosition,
                LeftIntakeVelocity,
                LeftIntakeAppliedVolts,
                LeftIntakeSupplyCurrent,
                LeftIntakeTorqueCurrent,
                LeftIntakeTempCelsius)
            .isOK();

    stats.LExtendPositionRads = LeftIntakePosition.getValueAsDouble();
    stats.LExtendVelocityRpm = LeftIntakeVelocity.getValueAsDouble();
    stats.LExtendAppliedVolts = LeftIntakeAppliedVolts.getValueAsDouble();
    stats.LExtendSupplyCurrentAmps = LeftIntakeSupplyCurrent.getValueAsDouble();
    stats.LExtendTorqueCurrentAmps = LeftIntakeTorqueCurrent.getValueAsDouble();
    stats.LExtendTempCelsius = LeftIntakeTempCelsius.getValueAsDouble();

    if(isSimulation){
      PhysicsSim.getInstance().run();
    }

    stats.MotorConnected =
        BaseStatusSignal.refreshAll(
                RightIntakePosition,
                RightIntakeVelocity,
                RightIntakeAppliedVolts,
                RightIntakeSupplyCurrent,
                RightIntakeTorqueCurrent,
                RightIntakeTempCelsius)
            .isOK();

    stats.RExtendPositionRads = RightIntakePosition.getValueAsDouble();
    stats.RExtendVelocityRpm = RightIntakeVelocity.getValueAsDouble();
    stats.RExtendAppliedVolts = RightIntakeAppliedVolts.getValueAsDouble();
    stats.RExtendSupplyCurrentAmps = RightIntakeSupplyCurrent.getValueAsDouble();
    stats.RExtendTorqueCurrentAmps = RightIntakeTorqueCurrent.getValueAsDouble();
    stats.RExtendTempCelsius = RightIntakeTempCelsius.getValueAsDouble();

    if(isSimulation){
      PhysicsSim.getInstance().run();
    }

    stats.MotorConnected =
        BaseStatusSignal.refreshAll(
                LeftRollerPosition,
                LeftRollerVelocity,
                LeftRollerAppliedVolts,
                LeftRollerSupplyCurrent,
                LeftRollerTorqueCurrent,
                LeftRollerTempCelsius)
            .isOK();

    stats.LRollerPositionRads = LeftRollerPosition.getValueAsDouble();
    stats.LRollerVelocityRpm = LeftRollerVelocity.getValueAsDouble();
    stats.LRollerAppliedVolts = LeftRollerAppliedVolts.getValueAsDouble();
    stats.LRollerSupplyCurrentAmps = LeftRollerSupplyCurrent.getValueAsDouble();
    stats.LRollerTorqueCurrentAmps = LeftRollerTorqueCurrent.getValueAsDouble();
    stats.LRollerTempCelsius = LeftRollerTempCelsius.getValueAsDouble();

    if(isSimulation){
      PhysicsSim.getInstance().run();
    }

    stats.MotorConnected =
        BaseStatusSignal.refreshAll(
                RightRollerPosition,
                RightRollerVelocity,
                RightRollerAppliedVolts,
                RightRollerSupplyCurrent,
                RightRollerTorqueCurrent,
                RightRollerTempCelsius)
            .isOK();

    stats.RRollerPositionRads = RightRollerPosition.getValueAsDouble();
    stats.RRollerVelocityRpm = RightRollerVelocity.getValueAsDouble();
    stats.RRollerAppliedVolts = RightRollerAppliedVolts.getValueAsDouble();
    stats.RRollerSupplyCurrentAmps = RightRollerSupplyCurrent.getValueAsDouble();
    stats.RRollerTorqueCurrentAmps = RightRollerTorqueCurrent.getValueAsDouble();
    stats.RRollerTempCelsius = RightRollerTempCelsius.getValueAsDouble();
  }



  /***************************************************************************
  * The runVelocity function can run (based on the controlMode Variable either:
  *     0-means run Position Voltage Control Mode  
  *     1-means run Position Torque Control Mode  
  *     2-means run Motion Magic Expo Voltage Control Mode  
  *     3-means run Motion Magic Expo Torque Control mode  
  ***************************************************************************/
  public void runPosition(double desiredRotations,int Side) {    
    
    //  double adjRotations = roundTo1Decimal(desiredRotations);
    double adjRotations = desiredRotations;
    if(Side == 1) {
        switch (controlPos) {
            case 0:      /* Use Position Voltage */
                m_LExtend.setControl(m_positionVoltage.withPosition(adjRotations));
            case 1:      /* Use Position Torque */
                m_LExtend.setControl(m_positionTorque.withPosition(adjRotations));
            case 2:      /* Use Motion Magic Position Voltage */
                m_LExtend.setControl(m_mmExpoVoltage.withPosition(adjRotations));
            case 3:      /* Use Motion Magic Position Torque */
                m_LExtend.setControl(m_mmExpoTorque.withPosition(adjRotations));
        }
    }else {   
        switch (controlPos) {
            case 0:      /* Use Position Voltage */
                m_RExtend.setControl(m_positionVoltage.withPosition(adjRotations));
            case 1:      /* Use Position Torque */
                m_RExtend.setControl(m_positionTorque.withPosition(adjRotations));
            case 2:      /* Use Motion Magic Position Voltage */
                m_RExtend.setControl(m_mmExpoVoltage.withPosition(adjRotations));
            case 3:      /* Use Motion Magic Position Torque */
                m_RExtend.setControl(m_mmExpoTorque.withPosition(adjRotations));
        }
    }
  }

  /*********************
  * The runVelocity function can run (based on the controlMode Variable either:
  *     0-means run Velocity Voltage Control Mode  
  *     1-means run Velocity Torque Control Mode  
  *     2-means run Motion Magic Voltage Control Mode  
  *     3-means run Motion Magic Torque Control mode  
  **********************/
  public void runVelocity(double RPM,int Side) {      
    //Convert rotations per minute into rotations per second 
    double desiredRotationsPerSecond = (RPM/60.0); 
    if(Side == 1) {
        switch (controlVel) {
            case 0:      /* Use velocity voltage */
                m_LRoller.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
            case 1:      /* Use velocity torque */
                m_LRoller.setControl(m_velocityTorque.withVelocity(desiredRotationsPerSecond));
            case 2:      /*Use Motion Magic Voltage */
                m_LRoller.setControl(m_mmVelocityVoltage.withPosition(desiredRotationsPerSecond));
            case 3:      /*Use Motion Magic Torque Current FOC */
                m_LRoller.setControl(m_mmVelocityTorque.withPosition(desiredRotationsPerSecond));   
      }
  }else {
        switch (controlVel) {
            case 0:      /* Use velocity voltage */
                m_RRoller.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
            case 1:      /* Use velocity torque */
                m_RRoller.setControl(m_velocityTorque.withVelocity(desiredRotationsPerSecond));
            case 2:      /*Use Motion Magic Voltage */
                m_RRoller.setControl(m_mmVelocityVoltage.withPosition(desiredRotationsPerSecond));
            case 3:      /*Use Motion Magic Torque Current FOC */
                m_RRoller.setControl(m_mmVelocityTorque.withPosition(desiredRotationsPerSecond));
        }
      }
   }
  }

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
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.sim.PhysicsSim;
import frc.robot.sim.PhysicsSimWithCan;


public class IntakeKraken {
  // Hardware
  private CANBus canBus = new CANBus(IntakeConstants.kBusName);
  private TalonFX m_Extend = new TalonFX(IntakeConstants.kIntakeMotorIDExtend, canBus);
  private TalonFX m_Roller = new TalonFX(IntakeConstants.kIntakeMotorIDRoller, canBus);
  private CANcoder m_CanCoder = new CANcoder(IntakeConstants.kIntakeEncoderID, canBus);
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
  
  private final StatusSignal<Angle> IntakePosition;
  private final StatusSignal<AngularVelocity> IntakeVelocity;
  private final StatusSignal<Voltage> IntakeAppliedVolts;
  private final StatusSignal<Current> IntakeSupplyCurrent;
  private final StatusSignal<Current> IntakeTorqueCurrent;
  private final StatusSignal<Temperature> IntakeTempCelsius;
  private final StatusSignal<Angle> IntakeEncoderPosition;
  private final StatusSignal<AngularVelocity> IntakeEncoderVelocity;

  private final StatusSignal<Angle> RollerPosition;
  private final StatusSignal<AngularVelocity> RollerVelocity;
  private final StatusSignal<Voltage> RollerAppliedVolts;
  private final StatusSignal<Current> RollerSupplyCurrent;
  private final StatusSignal<Current> RollerTorqueCurrent;
  private final StatusSignal<Temperature> RollerTempCelsius;

  private static IntakeKraken instance = null;

  /**************************************************************** 
  * Change this variable to select which motor control feature runs 
  *     0-means run Velocity Voltage Control Mode  
  *     1-means run Velocity Torque Control Mode  
  *     2-means run Motion Magic Voltage Control Mode  
  *     3-means run Motion Magic Torque Control mode  
  *****************************************************************/           
  private int controlVel = 0;
  private int controlPos = 0;
  private StatusCode statuscode = null;

  public IntakeKraken(){
    /**********************************
    * Confiure the shooter Motor 
    ***********************************/
    // Apply Configuration from Intake Constants to motor 
    statuscode = m_Extend.getConfigurator().apply(IntakeConstants.IntakeConfigs.IntakeExtendConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Left Extend shooter Motor ");
    } 
    statuscode = m_Roller.getConfigurator().apply(IntakeConstants.IntakeConfigs.RollerConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Left Roller shooter Motor ");
    } 
    statuscode = m_CanCoder.getConfigurator().apply(IntakeConstants.IntakeConfigs.IntakeEncoderConfigs(),1.0);
    if (!statuscode.isOK()) {
      System.out.println("Configuration Failed on LeftIntake CanCoder");
    }

    // Set signals
    IntakePosition = m_Extend.getPosition();
    IntakeVelocity = m_Extend.getVelocity();
    IntakeAppliedVolts = m_Extend.getMotorVoltage();
    IntakeSupplyCurrent = m_Extend.getSupplyCurrent();
    IntakeTorqueCurrent = m_Extend.getTorqueCurrent();
    IntakeTempCelsius = m_Extend.getDeviceTemp();
    IntakeEncoderPosition = m_CanCoder.getPosition();
    IntakeEncoderVelocity = m_CanCoder.getVelocity();
  
    RollerPosition = m_Roller.getPosition();
    RollerVelocity = m_Roller.getVelocity();
    RollerAppliedVolts = m_Roller.getMotorVoltage();
    RollerSupplyCurrent = m_Roller.getSupplyCurrent();
    RollerTorqueCurrent = m_Roller.getTorqueCurrent();
    RollerTempCelsius = m_Roller.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        IntakePosition,
        IntakeVelocity,
        IntakeAppliedVolts,
        IntakeSupplyCurrent,
        IntakeTorqueCurrent,
        IntakeTempCelsius,
        IntakeEncoderPosition,
        IntakeEncoderVelocity,
        RollerPosition,
        RollerVelocity,
        RollerAppliedVolts,
        RollerSupplyCurrent,
        RollerTorqueCurrent,
        RollerTempCelsius
    );

    /* Optimize out the other signals, since they're not useful */
    m_Extend.optimizeBusUtilization();
      m_Roller.optimizeBusUtilization();

    /* Defines the motor to the Physics Simulation-only in Simulation */
    if(Utils.isSimulation()){
      PhysicsSim.getInstance().addTalonFX(m_Extend, 0.001);
      PhysicsSim.getInstance().addTalonFX(m_Roller, 0.001);
      }
    }

  public static IntakeKraken getInstance() {
    return instance = instance == null ? new IntakeKraken() : instance;
  }

  public void updateStats(IntakeStats stats) {
    if(Utils.isSimulation()){
      PhysicsSim.getInstance().run();
    }

    stats.MotorConnected =
        BaseStatusSignal.refreshAll(
                IntakePosition,
                IntakeVelocity,
                IntakeAppliedVolts,
                IntakeSupplyCurrent,
                IntakeTorqueCurrent,
                IntakeTempCelsius,
                IntakeEncoderPosition,
                IntakeEncoderVelocity,
                RollerPosition,
                RollerVelocity,
                RollerAppliedVolts,
                RollerSupplyCurrent,
                RollerTorqueCurrent,
                RollerTempCelsius)
            .isOK();

    stats.ExtendPositionRads = IntakePosition.getValueAsDouble();
    stats.ExtendVelocityRpm = IntakeVelocity.getValueAsDouble();
    stats.ExtendAppliedVolts = IntakeAppliedVolts.getValueAsDouble();
    stats.ExtendSupplyCurrentAmps = IntakeSupplyCurrent.getValueAsDouble();
    stats.ExtendTorqueCurrentAmps = IntakeTorqueCurrent.getValueAsDouble();
    stats.ExtendTempCelsius = IntakeTempCelsius.getValueAsDouble();
    stats.IntakeEncoderPosition = IntakeEncoderPosition.getValueAsDouble();
    stats.IntakeEncoderVelocity = IntakeEncoderVelocity.getValueAsDouble();
    stats.RollerPositionRads = RollerPosition.getValueAsDouble();
    stats.RollerVelocityRpm = RollerVelocity.getValueAsDouble();
    stats.RollerAppliedVolts = RollerAppliedVolts.getValueAsDouble();
    stats.RollerSupplyCurrentAmps = RollerSupplyCurrent.getValueAsDouble();
    stats.RollerTorqueCurrentAmps = RollerTorqueCurrent.getValueAsDouble();
    stats.RollerTempCelsius = RollerTempCelsius.getValueAsDouble();
  }

  /***************************************************************************
  * The runVelocity function can run (based on the controlMode Variable either:
  *     0-means run Position Voltage Control Mode  
  *     1-means run Position Torque Control Mode  
  *     2-means run Motion Magic Expo Voltage Control Mode  
  *     3-means run Motion Magic Expo Torque Control mode  
  ***************************************************************************/
  
  public void test(){
    m_Extend.setControl(m_positionVoltage.withPosition(10));
  }
  
  public void runPosition(double desiredRotations,int Side) {     
    
    if(Side == 1) {
        switch (controlPos) {
           case 0:      /* Use Position Voltage */
                m_Extend.setControl(m_positionVoltage.withPosition(desiredRotations));
                break;
           case 1:      /* Use Position Torque */
                m_Extend.setControl(m_positionTorque.withPosition(desiredRotations));
                break;
           case 2:      /* Use Motion Magic Position Voltage */
                m_Extend.setControl(m_mmExpoVoltage.withPosition(desiredRotations));
                break;
           case 3:      /* Use Motion Magic Position Torque */
                m_Extend.setControl(m_mmExpoTorque.withPosition(desiredRotations));
                break;
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
                m_Roller.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
                break;
            case 1:      /* Use velocity torque */
                m_Roller.setControl(m_velocityTorque.withVelocity(desiredRotationsPerSecond));
                break;
            case 2:      /*Use Motion Magic Voltage */
                m_Roller.setControl(m_mmVelocityVoltage.withPosition(desiredRotationsPerSecond));
                break;
            case 3:      /*Use Motion Magic Torque Current FOC */
                m_Roller.setControl(m_mmVelocityTorque.withPosition(desiredRotationsPerSecond)); 
                break;  
        }
      }
   }
   public void stop(){
        m_Roller.setControl(m_brake);
    }
}

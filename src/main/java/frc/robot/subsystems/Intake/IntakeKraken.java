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


public class IntakeKraken {
  // Hardware
  private CANBus canBus = new CANBus(IntakeConstants.kBusName);
  private TalonFX m_LExtend = new TalonFX(IntakeConstants.kIntakeMotorIDLExtend, canBus);
  private TalonFX m_LRoller = new TalonFX(IntakeConstants.kIntakeMotorIDLRoller, canBus);
  private CANcoder m_LeftCanCoder = new CANcoder(IntakeConstants.kLeftIntakeEncoderID, canBus);
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
  private final StatusSignal<Angle> LeftIntakeEncoderPosition;
  private final StatusSignal<AngularVelocity> LeftIntakeEncoderVelocity;

  private final StatusSignal<Angle> LeftRollerPosition;
  private final StatusSignal<AngularVelocity> LeftRollerVelocity;
  private final StatusSignal<Voltage> LeftRollerAppliedVolts;
  private final StatusSignal<Current> LeftRollerSupplyCurrent;
  private final StatusSignal<Current> LeftRollerTorqueCurrent;
  private final StatusSignal<Temperature> LeftRollerTempCelsius;

  private static IntakeKraken instance = null;
  private boolean isSimulation = false; 

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
    statuscode = m_LExtend.getConfigurator().apply(IntakeConstants.IntakeConfigs.LeftIntakeExtendConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Left Extend shooter Motor ");
    } 
    statuscode = m_LRoller.getConfigurator().apply(IntakeConstants.IntakeConfigs.LeftRollerConfigs(), 1.0); 
    if(!statuscode.isOK()) { 
      System.out.println("Configuration Failed on Left Roller shooter Motor ");
    } 
    statuscode = m_LeftCanCoder.getConfigurator().apply(IntakeConstants.IntakeConfigs.LeftIntakeEncoderConfigs(),1.0);
    if (!statuscode.isOK()) {
      System.out.println("Configuration Failed on LeftIntake CanCoder");
    }

    // Set signals
    LeftIntakePosition = m_LExtend.getPosition();
    LeftIntakeVelocity = m_LExtend.getVelocity();
    LeftIntakeAppliedVolts = m_LExtend.getMotorVoltage();
    LeftIntakeSupplyCurrent = m_LExtend.getSupplyCurrent();
    LeftIntakeTorqueCurrent = m_LExtend.getTorqueCurrent();
    LeftIntakeTempCelsius = m_LExtend.getDeviceTemp();
    LeftIntakeEncoderPosition = m_LeftCanCoder.getPosition();
    LeftIntakeEncoderVelocity = m_LeftCanCoder.getVelocity();
  
    LeftRollerPosition = m_LRoller.getPosition();
    LeftRollerVelocity = m_LRoller.getVelocity();
    LeftRollerAppliedVolts = m_LRoller.getMotorVoltage();
    LeftRollerSupplyCurrent = m_LRoller.getSupplyCurrent();
    LeftRollerTorqueCurrent = m_LRoller.getTorqueCurrent();
    LeftRollerTempCelsius = m_LRoller.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
          LeftIntakePosition,
        LeftIntakeVelocity,
        LeftIntakeAppliedVolts,
        LeftIntakeSupplyCurrent,
        LeftIntakeTorqueCurrent,
        LeftIntakeTempCelsius,
        LeftIntakeEncoderPosition,
        LeftIntakeEncoderVelocity,
          LeftRollerPosition,
        LeftRollerVelocity,
        LeftRollerAppliedVolts,
        LeftRollerSupplyCurrent,
        LeftRollerTorqueCurrent,
        LeftRollerTempCelsius
    );

    /* Optimize out the other signals, since they're not useful */
    m_LExtend.optimizeBusUtilization();
      m_LRoller.optimizeBusUtilization();

    /* Defines the motor to the Physics Simulation-only in Simulation */
    if(Utils.isSimulation()){
      isSimulation = true; 
      PhysicsSim.getInstance().addTalonFX(m_LExtend, 0.001);
      PhysicsSim.getInstance().addTalonFX(m_LRoller, 0.001);
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
                LeftIntakeTempCelsius,
                LeftIntakeEncoderPosition,
                LeftIntakeEncoderVelocity,
                LeftRollerPosition,
                LeftRollerVelocity,
                LeftRollerAppliedVolts,
                LeftRollerSupplyCurrent,
                LeftRollerTorqueCurrent,
                LeftRollerTempCelsius)
            .isOK();

    stats.LExtendPositionRads = LeftIntakePosition.getValueAsDouble();
    stats.LExtendVelocityRpm = LeftIntakeVelocity.getValueAsDouble();
    stats.LExtendAppliedVolts = LeftIntakeAppliedVolts.getValueAsDouble();
    stats.LExtendSupplyCurrentAmps = LeftIntakeSupplyCurrent.getValueAsDouble();
    stats.LExtendTorqueCurrentAmps = LeftIntakeTorqueCurrent.getValueAsDouble();
    stats.LExtendTempCelsius = LeftIntakeTempCelsius.getValueAsDouble();
    stats.LeftIntakeEncoderPosition = LeftIntakeEncoderPosition.getValueAsDouble();
    stats.LeftIntakeEncoderVelocity = LeftIntakeEncoderVelocity.getValueAsDouble();
    stats.LRollerPositionRads = LeftRollerPosition.getValueAsDouble();
    stats.LRollerVelocityRpm = LeftRollerVelocity.getValueAsDouble();
    stats.LRollerAppliedVolts = LeftRollerAppliedVolts.getValueAsDouble();
    stats.LRollerSupplyCurrentAmps = LeftRollerSupplyCurrent.getValueAsDouble();
    stats.LRollerTorqueCurrentAmps = LeftRollerTorqueCurrent.getValueAsDouble();
    stats.LRollerTempCelsius = LeftRollerTempCelsius.getValueAsDouble();
  }



  /***************************************************************************
  * The runVelocity function can run (based on the controlMode Variable either:
  *     0-means run Position Voltage Control Mode  
  *     1-means run Position Torque Control Mode  
  *     2-means run Motion Magic Expo Voltage Control Mode  
  *     3-means run Motion Magic Expo Torque Control mode  
  ***************************************************************************/
  
  public void test(){
    m_LExtend.setControl(m_positionVoltage.withPosition(10));
  }
  
  public void runPosition(double desiredRotations,int Side) {     
    
    if(Side == 1) {
        switch (controlPos) {
           case 0:      /* Use Position Voltage */
                m_LExtend.setControl(m_positionVoltage.withPosition(desiredRotations));
                break;
           case 1:      /* Use Position Torque */
                m_LExtend.setControl(m_positionTorque.withPosition(desiredRotations));
                break;
           case 2:      /* Use Motion Magic Position Voltage */
                m_LExtend.setControl(m_mmExpoVoltage.withPosition(desiredRotations));
                break;
           case 3:      /* Use Motion Magic Position Torque */
                m_LExtend.setControl(m_mmExpoTorque.withPosition(desiredRotations));
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
                m_LRoller.setControl(m_velocityVoltage.withVelocity(desiredRotationsPerSecond));
                break;
            case 1:      /* Use velocity torque */
                m_LRoller.setControl(m_velocityTorque.withVelocity(desiredRotationsPerSecond));
                break;
            case 2:      /*Use Motion Magic Voltage */
                m_LRoller.setControl(m_mmVelocityVoltage.withPosition(desiredRotationsPerSecond));
                break;
            case 3:      /*Use Motion Magic Torque Current FOC */
                m_LRoller.setControl(m_mmVelocityTorque.withPosition(desiredRotationsPerSecond)); 
                break;  
        }
      }
   }
   public void stop(){
        m_LRoller.setControl(m_brake);
    }
}

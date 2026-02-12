package frc.robot.subsystems.Climber;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;

import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.Climber.Climber.ClimberStats;

public class ClimberKraken {

    private CANBus canbus = new CANBus(ClimberConstants.kBusName);
    private TalonFX m_talonFX = new TalonFX(ClimberConstants.kClimbermotorID, canbus);
    private TalonFX m_talonFX2 = new TalonFX(ClimberConstants.kClimberspinnerID, canbus);
    private static ClimberKraken instance = null;
    private boolean isSimulation = false;

    private final StatusSignal<Angle> climberPosition;
    private final StatusSignal<AngularVelocity> climberVelocity;
    private final StatusSignal<Voltage> climberAppliedVolts;
    private final StatusSignal<Current> climberSupplyCurrent;
    private final StatusSignal<Current> climberTorqueCurrent;
    private final StatusSignal<Temperature> climberTempCelsius;
    private final StatusSignal<Angle> spinnerPosition;
    private final StatusSignal<AngularVelocity> spinnerVelocity;
    private final StatusSignal<Voltage> spinnerAppliedVolts;
    private final StatusSignal<Current> spinnerSupplyCurrent;
    private final StatusSignal<Current> spinnerTorqueCurrent;
    private final StatusSignal<Temperature> spinnerTempCelsius;
    /**************************************************************** 
     *z Change this variable to select which motor control feature runs 
    *     0-means run Position Voltage Control Mode  
    *     1-means run Position Torque Control Mode  
    *     2-means run Motion Magic Expo Position Voltage Control Mode  
    *     3-means run Motion Magic Expo Position Torque Control mode  
    *****************************************************************/           
    private int controlMode = 3; /* 0-PositionVoltage, 1-PositionTorque, 2-MotionMagic Position Voltage, 3-MotionMagic Position Torque */ 
      
    //Control 
    private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
    private final PositionVoltage m_spinpositionVoltage = new PositionVoltage(0).withSlot(1);
    private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);
    private final MotionMagicExpoVoltage m_mmExpoVoltage = new MotionMagicExpoVoltage(0).withSlot(2);
    private final MotionMagicExpoVoltage m_mmspinExpoVoltage = new MotionMagicExpoVoltage(0).withSlot(1);
    private final MotionMagicExpoTorqueCurrentFOC m_mmExpoTorque = new MotionMagicExpoTorqueCurrentFOC(0.0).withSlot(1);
    private final MotionMagicExpoTorqueCurrentFOC m_mmspinExpoTorque = new MotionMagicExpoTorqueCurrentFOC(0.0).withSlot(2);

    private final NeutralOut m_brake = new NeutralOut();   /* Keep a neutral out so we can disable the motor */


    public static ClimberKraken getInstance() {
        return instance = instance == null ? new ClimberKraken() : instance;
    }

    public void updateStats(ClimberStats stats){
       
        if(isSimulation){
            PhysicsSim.getInstance().run();
        }
        stats.MotorConnected =
            BaseStatusSignal.refreshAll(
            climberPosition,
            climberVelocity,
            climberAppliedVolts,
            climberSupplyCurrent,
            climberTorqueCurrent,
            climberTempCelsius,
            spinnerPosition,
            spinnerVelocity,
            spinnerAppliedVolts,
            spinnerSupplyCurrent,
            spinnerTorqueCurrent,
            spinnerTempCelsius)
            .isOK();

            stats.climberPositionRads = climberPosition.getValueAsDouble();
            stats.climberVelocityRpm = climberVelocity.getValueAsDouble();
            stats.climberAppliedVolt = climberAppliedVolts.getValueAsDouble();
            stats.climberSupplyCurrentAmps = climberSupplyCurrent.getValueAsDouble();
            stats.climberTorqueCurrentApls = climberTorqueCurrent.getValueAsDouble();
            stats.climberTempCelsius = climberTempCelsius.getValueAsDouble();
            stats.spinnerPositionRads = spinnerPosition.getValueAsDouble();
            stats.spinnerVelocityRpm = spinnerVelocity.getValueAsDouble();
            stats.spinnerAppliedVolt = spinnerAppliedVolts.getValueAsDouble();
            stats.spinnerSupplyCurrentAmps = spinnerSupplyCurrent.getValueAsDouble();
            stats.spinnerTorqueCurrentApls = spinnerTorqueCurrent.getValueAsDouble();
            stats.spinnerTempCelsius = spinnerTempCelsius.getValueAsDouble();
    }


    /***************************************************************************
     * The runVelocity function can run (based on the controlMode Variable either:
    *     0-means run Position Voltage Control Mode  
    *     1-means run Position Torque Control Mode  
    *     2-means run Motion Magic Expo Voltage Control Mode  
    *     3-means run Motion Magic Expo Torque Control mode  
    ***************************************************************************/
    
    public void runManual(double percentOutputElbow, double percentOutputWrist) {
        m_talonFX.setControl(new DutyCycleOut(percentOutputElbow));
        m_talonFX2.setControl(new DutyCycleOut(percentOutputWrist));
    }

    public void runPosition(double desiredRotations) {
        switch (controlMode) {
        case 0:      /* Use Position Voltage */
            m_talonFX.setControl(m_positionVoltage.withPosition(desiredRotations));
        case 1:      /* Use Position Torque */
            m_talonFX.setControl(m_positionTorque.withPosition(desiredRotations));
        case 2:      /* Use Motion Magic Position Voltage */
            m_talonFX.setControl(m_mmExpoVoltage.withPosition(desiredRotations));
        case 3:      /* Use Motion Magic Position Torque */
            m_talonFX.setControl(m_mmExpoTorque.withPosition(desiredRotations));
        }
    }
    public void spinrunPosition(double desiredRotations) {
        switch (controlMode) {
            case 0:
                m_talonFX2.setControl(m_spinpositionVoltage.withPosition(desiredRotations));
            case 2:
                m_talonFX2.setControl(m_mmspinExpoVoltage.withPosition(desiredRotations));
            case 3:
                m_talonFX2.setControl(m_mmspinExpoTorque.withPosition(desiredRotations));
        }
    }


    public ClimberKraken() {
        StatusCode statuscode = m_talonFX.getConfigurator().apply(ClimberConstants.ClimberConfigs.climberWheelConfigs(), 1.0); 
        StatusCode statuscode2 = m_talonFX2.getConfigurator().apply(ClimberConstants.ClimberConfigs.climberWheelConfigs(), 1.0);
       
        if (!statuscode.isOK()) {
            System.out.println("Configuration Failed on Climber Motor");
        }
        if (!statuscode2.isOK()) {
            System.out.println("Configuration Failed on Climber Motor");
        }
        climberPosition = m_talonFX.getPosition();
        climberVelocity = m_talonFX.getVelocity();
        climberAppliedVolts = m_talonFX.getMotorVoltage();
        climberSupplyCurrent = m_talonFX.getSupplyCurrent();
        climberTorqueCurrent = m_talonFX.getTorqueCurrent();
        climberTempCelsius = m_talonFX.getDeviceTemp();
        spinnerPosition = m_talonFX2.getPosition();
        spinnerVelocity = m_talonFX2.getVelocity();
        spinnerAppliedVolts = m_talonFX2.getMotorVoltage();
        spinnerSupplyCurrent = m_talonFX2.getSupplyCurrent();
        spinnerTorqueCurrent = m_talonFX2.getTorqueCurrent();
        spinnerTempCelsius = m_talonFX2.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        climberPosition,
        climberVelocity,
        climberAppliedVolts,
        climberSupplyCurrent,
        climberTorqueCurrent,
        climberTempCelsius,
        spinnerPosition,
        spinnerVelocity,
        spinnerAppliedVolts,
        spinnerSupplyCurrent,
        spinnerTorqueCurrent,
        spinnerTempCelsius
    );
    
    m_talonFX.optimizeBusUtilization();
    m_talonFX2.optimizeBusUtilization();

    if(Utils.isSimulation()){
        isSimulation = true;
        PhysicsSim.getInstance().addTalonFX(m_talonFX, 0.01);
        PhysicsSim.getInstance().addTalonFX(m_talonFX2, 0.01);
        }

    }
}

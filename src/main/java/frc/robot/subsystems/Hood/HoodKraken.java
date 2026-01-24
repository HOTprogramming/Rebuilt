package frc.robot.subsystems.Hood;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.Hood.Hood.hoodStats;

public class HoodKraken {
    private static HoodKraken instance = null;
    private boolean isSimulation = false;

    /**************************************************************** 
    * Change this variable to select which motor control feature runs 
    *     0-means run Position Voltage Control Mode  
    *     1-means run Position Torque Control Mode  
    *****************************************************************/           
    private int controlMode = 0;

    // Initialize Motors //
    private TalonFX m_talonFX = new TalonFX(HoodConstants.kHoodMotorID, HoodConstants.canbus);

    //Control 
    private final PositionVoltage m_positionVoltage = new PositionVoltage(0).withSlot(0);
    private final PositionTorqueCurrentFOC m_positionTorque = new PositionTorqueCurrentFOC(0).withSlot(1);

    private final NeutralOut m_brake = new NeutralOut();   /* Keep a neutral out so we can disable the motor */

    // Status Signals //
    private final StatusSignal<Angle> hoodPosition;
    private final StatusSignal<AngularVelocity> hoodVelocity;
    private final StatusSignal<Voltage> hoodAppliedVolts;
    private final StatusSignal<Current> hoodSupplyCurrent;
    private final StatusSignal<Current> hoodTorqueCurrent;
    private final StatusSignal<Temperature> hoodTempCelsius;

    public HoodKraken() {
        
        /**********************************
        * Confiure the Hood Motor 
        ***********************************/
        // Apply Configuration from Hood Constants to motor //
        StatusCode statuscode = m_talonFX.getConfigurator().apply(HoodConstants.HoodConfigs.hoodMotorConfigs(), 1.0); 
        if(!statuscode.isOK()) { 
            System.out.println("Configuration Failed on Hood Motor");
        }
        
        /* Defines the motor to the Physics Simulation-only in Simulation */
        if(Utils.isSimulation()){
            isSimulation = true; 
            PhysicsSim.getInstance().addTalonFX(m_talonFX, 0.001);
        }

        // Initialize Telemetry data //
        hoodPosition = m_talonFX.getPosition();
        hoodVelocity = m_talonFX.getVelocity();
        hoodAppliedVolts = m_talonFX.getMotorVoltage();
        hoodSupplyCurrent = m_talonFX.getSupplyCurrent();
        hoodTorqueCurrent = m_talonFX.getTorqueCurrent();
        hoodTempCelsius = m_talonFX.getDeviceTemp();

        BaseStatusSignal.setUpdateFrequencyForAll(
            100.0,
            hoodPosition,
            hoodVelocity,
            hoodAppliedVolts,
            hoodSupplyCurrent,
            hoodTorqueCurrent,
            hoodTempCelsius
        );

        /* Optimize out the other signals, since they're not useful */
        m_talonFX.optimizeBusUtilization();

        /* Defines the motor to the Physics Simulation-only in Simulation */
        if(Utils.isSimulation()){
            isSimulation = true; 
            PhysicsSim.getInstance().addTalonFX(m_talonFX, 0.001);
        }
    }

    /*****************************************************************************
   * Return an instance of this class - Ensures that there is a single instance   
   ****************************************************************************/
    public static HoodKraken getInstance() {
        return instance = instance == null ? new HoodKraken() : instance;
    }

    public void updateStats(hoodStats stats) {
        if(isSimulation) {
            PhysicsSim.getInstance().run();
        }
    }

    /***************************************************************************
  * The runVelocity function can run (based on the controlMode Variable either:
  *     0-means run Position Voltage Control Mode  
  *     1-means run Position Torque Control Mode  
  ***************************************************************************/
    public void runPosition(double desiredRotations) {      
        switch (controlMode) {
        case 0:      /* Use Position Voltage */
            m_talonFX.setControl(m_positionVoltage.withPosition(desiredRotations));
        case 1:      /* Use Position Torque */
            m_talonFX.setControl(m_positionTorque.withPosition(desiredRotations));
        }
    }
}
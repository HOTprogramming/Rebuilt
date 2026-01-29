package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.RPM;

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
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.sim.PhysicsSim;

import frc.robot.subsystems.Indexer.Index.IndexStats;

public class IndexKraken {

    private CANBus canbus = new CANBus(IndexConstants.kBusName);
    private TalonFX m_talonFX = new TalonFX(IndexConstants.kLeftKickID, canbus);
    private TalonFX m_talonFX2 = new TalonFX(IndexConstants.kRightKickID, canbus);
    private TalonFX m_TalonFX3 = new TalonFX(IndexConstants.kFloorID);
    private TalonFX m_TalonFX4 = new TalonFX(IndexConstants.kLoaderID);
    
    private static IndexKraken instance = null;
    private boolean isSimulation = false;

    private final StatusSignal<Angle> indexPosition;
    private final StatusSignal<AngularVelocity> indexVelocity;
    private final StatusSignal<Voltage> indexAppliedVolts;
    private final StatusSignal<Current> indexSupplyCurrent;
    private final StatusSignal<Current> indexTorqueCurrent;
    private final StatusSignal<Temperature> indexTempCelsius;
        /**************************************************************** 
     * Change this variable to select which motor control feature runs 
    *     0-means run Position Voltage Control Mode  
    *     1-means run Position Torque Control Mode  
    *     2-means run Motion Magic Expo Position Voltage Control Mode  
    *     3-means run Motion Magic Expo Position Torque Control mode  
    *****************************************************************/           
    private int controlMode = 0; /* 0-PositionVoltage, 1-PositionTorque, 2-MotionMagic Position Voltage, 3-MotionMagic Position Torque */ 
  
    private final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0).withSlot(0);

    public static IndexKraken getInstance() {
        return instance = instance == null ? new IndexKraken() : instance;
    }
    public void updateStats(IndexStats stats){
       
        if(isSimulation){
            PhysicsSim.getInstance().run();
        }
        stats.MotorConnected =
            BaseStatusSignal.refreshAll(
            indexPosition,
            indexVelocity,
            indexAppliedVolts,
            indexSupplyCurrent,
            indexTorqueCurrent,
            indexTempCelsius)
            .isOK();

            stats.indexPositionRads = indexPosition.getValueAsDouble();
            stats.indexVelocityRpm = indexVelocity.getValueAsDouble();
            stats.indexAppliedVolt = indexAppliedVolts.getValueAsDouble();
            stats.indexSupplyCurrentAmps = indexSupplyCurrent.getValueAsDouble();
            stats.indexTorqueCurrentApls = indexTorqueCurrent.getValueAsDouble();
            stats.indexTempCelsius = indexTempCelsius.getValueAsDouble();
    }
    public IndexKraken() {
        indexPosition = m_talonFX.getPosition();
        indexVelocity = m_talonFX.getVelocity();
        indexAppliedVolts = m_talonFX.getMotorVoltage();
        indexSupplyCurrent = m_talonFX.getSupplyCurrent();
        indexTorqueCurrent = m_talonFX.getTorqueCurrent();
        indexTempCelsius = m_talonFX.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        indexPosition,
        indexVelocity,
        indexAppliedVolts,
        indexSupplyCurrent,
        indexTorqueCurrent,
        indexTempCelsius
    );}
    // for the kicker
    public void runKick2(double speed) {
            double desiredRotationsPerSecond = 60.0; //IMPORTANT
        switch (controlMode) {
            case 0:
            m_talonFX.setControl(m_VelocityVoltage.withVelocity(desiredRotationsPerSecond));
        }
    }
    public void runKick(double speed) {
        double desiredRotationsPerSecond = 60.0; //IMPORTANT
        switch (controlMode) {
            case 0:
            m_talonFX2.setControl(m_VelocityVoltage.withVelocity(desiredRotationsPerSecond));
    }
    }
    //end of kicker, start of floor
    public void runFloor(double speed) {
        double desiredRotationsPerSecond = 60.0; //IMPORTANT
        switch (controlMode) {
            case 0:
            m_TalonFX3.setControl(m_VelocityVoltage.withVelocity(desiredRotationsPerSecond));
            m_TalonFX4.setControl(m_VelocityVoltage.withVelocity(-desiredRotationsPerSecond));
        }
    }
    //end of floor, start of loader
    public void runLoad(double speed) {
        double desiredRotationsPerSecond = 60.0; //IMPORTANT
        switch (controlMode) {
            case 0:
            m_TalonFX4.setControl(m_VelocityVoltage.withVelocity(-desiredRotationsPerSecond));
    }
}
}

package frc.robot.subsystems.Indexer;

import static edu.wpi.first.units.Units.RPM;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.math.StateSpaceUtil;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Frequency;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.controls.MotionMagicExpoTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicExpoVoltage;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;

import frc.robot.sim.PhysicsSim;
import frc.robot.subsystems.Indexer.IndexConstants;
import frc.robot.subsystems.Indexer.Index.IndexStats;

public class IndexKraken {

    private CANBus canbus = new CANBus(IndexConstants.kBusName);
    private TalonFX m_LeftKick = new TalonFX(IndexConstants.kLeftKickID, canbus);
    private TalonFX m_RightKick = new TalonFX(IndexConstants.kRightKickID, canbus);
    private TalonFX m_Floor = new TalonFX(IndexConstants.kFloorID, canbus);
    private TalonFX m_Loader = new TalonFX(IndexConstants.kLoaderID, canbus);
    
    private static IndexKraken instance = null;
    private boolean isSimulation = false;

    private final StatusSignal<Angle> kickerLeftPosition;
    private final StatusSignal<AngularVelocity> kickerLeftVelocity;
    private final StatusSignal<Voltage> kickerLeftAppliedVolts;
    private final StatusSignal<Current> kickerLeftSupplyCurrent;
    private final StatusSignal<Current> kickerLeftTorqueCurrent;
    private final StatusSignal<Temperature> kickerLeftTempCelsius;
    private final StatusSignal<Angle> kickerRightPosition;
    private final StatusSignal<AngularVelocity> kickerRightVelocity;
    private final StatusSignal<Voltage> kickerRightAppliedVolts;
    private final StatusSignal<Current> kickerRightSupplyCurrent;
    private final StatusSignal<Current> kickerRightTorqueCurrent;
    private final StatusSignal<Temperature> kickerRightTempCelsius;
    private final StatusSignal<Angle> FloorPosition;
    private final StatusSignal<AngularVelocity> FloorVelocity;
    private final StatusSignal<Voltage> FloorAppliedVolts;
    private final StatusSignal<Current> FloorSupplyCurrent;
    private final StatusSignal<Current> FloorTorqueCurrent;
    private final StatusSignal<Temperature> FloorTempCelsius;
    private final StatusSignal<Angle> LoadPosition;
    private final StatusSignal<AngularVelocity> LoadVelocity;
    private final StatusSignal<Voltage> LoadAppliedVolts;
    private final StatusSignal<Current> LoadSupplyCurrent;
    private final StatusSignal<Current> LoadTorqueCurrent;
    private final StatusSignal<Temperature> LoadTempCelsius;
        /**************************************************************** 
     * Change this variable to select which motor control feature runs 
    *     0-means run Position Voltage Control Mode  
    *     1-means run Position Torque Control Mode  
    *     2-means run Motion Magic Expo Position Voltage Control Mode  
    *     3-means run Motion Magic Expo Position Torque Control mode  
    *****************************************************************/           
    private int controlMode = 0; /* 0-PositionVoltage, 1-PositionTorque, 2-MotionMagic Position Voltage, 3-MotionMagic Position Torque */ 
  
    private final VelocityVoltage m_VelocityVoltage = new VelocityVoltage(0).withSlot(0);
    private final NeutralOut m_brake = new NeutralOut();
    
    public static IndexKraken getInstance() {
        return instance = instance == null ? new IndexKraken() : instance;
    }
    public void updateStats(IndexStats stats){
       
        if(isSimulation){
            PhysicsSim.getInstance().run();
        }
        stats.MotorConnected =
            BaseStatusSignal.refreshAll(
            kickerLeftPosition,
            kickerLeftVelocity,
            kickerLeftAppliedVolts,
            kickerLeftSupplyCurrent,
            kickerLeftTorqueCurrent,
            kickerLeftTempCelsius,
            kickerRightPosition,
            kickerRightVelocity,
            kickerRightAppliedVolts,
            kickerRightSupplyCurrent,
            kickerRightTorqueCurrent,
            kickerRightTempCelsius,
            FloorPosition,
            FloorVelocity,
            FloorAppliedVolts,
            FloorSupplyCurrent,
            FloorTorqueCurrent,
            FloorTempCelsius,
            LoadPosition,
            LoadVelocity,
            LoadAppliedVolts,
            LoadSupplyCurrent,
            LoadTorqueCurrent,
            LoadTempCelsius)
            .isOK();

            stats.kickerLeftPositionRads = kickerLeftPosition.getValueAsDouble();
            stats.kickerLeftVelocityRpm = kickerLeftVelocity.getValueAsDouble();
            stats.kickerLeftAppliedVolt = kickerLeftAppliedVolts.getValueAsDouble();
            stats.kickerLeftSupplyCurrentAmps = kickerLeftSupplyCurrent.getValueAsDouble();
            stats.kickerLeftTorqueCurrentApls = kickerLeftTorqueCurrent.getValueAsDouble();
            stats.kickerLeftTempCelsius = kickerLeftTempCelsius.getValueAsDouble();
            stats.kickerRightPositionRads = kickerRightPosition.getValueAsDouble();
            stats.kickerRightVelocityRpm = kickerRightVelocity.getValueAsDouble();
            stats.kickerRightAppliedVolt = kickerRightAppliedVolts.getValueAsDouble();
            stats.kickerRightSupplyCurrentAmps = kickerRightSupplyCurrent.getValueAsDouble();
            stats.kickerRightTorqueCurrentApls = kickerRightTorqueCurrent.getValueAsDouble();
            stats.kickerRightTempCelsius = kickerRightTempCelsius.getValueAsDouble();
            stats.FloorPositionRads = FloorPosition.getValueAsDouble();
            stats.FloorVelocityRpm = FloorVelocity.getValueAsDouble();
            stats.FloorAppliedVolt = FloorAppliedVolts.getValueAsDouble();
            stats.FloorSupplyCurrentAmps = FloorSupplyCurrent.getValueAsDouble();
            stats.FloorTorqueCurrentApls = FloorTorqueCurrent.getValueAsDouble();
            stats.FloorTempCelsius = FloorTempCelsius.getValueAsDouble();
            stats.LoadPositionRads = LoadPosition.getValueAsDouble();
            stats.LoadVelocityRpm = LoadVelocity.getValueAsDouble();
            stats.LoadAppliedVolt = LoadAppliedVolts.getValueAsDouble();
            stats.LoadSupplyCurrentAmps = LoadSupplyCurrent.getValueAsDouble();
            stats.LoadTorqueCurrentApls = LoadTorqueCurrent.getValueAsDouble();
            stats.LoadTempCelsius = LoadTempCelsius.getValueAsDouble();

    }
    public IndexKraken() {
        StatusCode statuscode = m_LeftKick.getConfigurator().apply(IndexConstants.IndexConfigs.indexWheelConfigs()
        .withMotorOutput( new MotorOutputConfigs()
        .withInverted(InvertedValue.Clockwise_Positive)), 1.0); 

        if (!statuscode.isOK()) {
            System.out.println("Configuration Failed on Index Kicker Left Motor");

        }

        StatusCode statuscode2 = m_RightKick.getConfigurator().apply(IndexConstants.IndexConfigs.indexWheelConfigs()
        .withMotorOutput( new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)), 1.0);
       
        if (!statuscode2.isOK()) {
            System.out.println("Configuration Failed on Index Kicker Right Motor");
        }

        StatusCode statuscode3 = m_Floor.getConfigurator().apply(IndexConstants.IndexConfigs.indexWheelConfigs()
        .withMotorOutput( new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)), 1.0);
       
        if (!statuscode3.isOK()) {
            System.out.println("Configuration Failed on Index Floor Motor");
        }

        StatusCode statuscode4 = m_Loader.getConfigurator().apply(IndexConstants.IndexConfigs.indexWheelConfigs()
        .withMotorOutput( new MotorOutputConfigs()
        .withInverted(InvertedValue.CounterClockwise_Positive)), 1.0);
       
        if (!statuscode4.isOK()) {
            System.out.println("Configuration Failed on Index Loader Motor");
        }


        kickerLeftPosition = m_LeftKick.getPosition();
        kickerLeftVelocity = m_LeftKick.getVelocity();
        kickerLeftAppliedVolts = m_LeftKick.getMotorVoltage();
        kickerLeftSupplyCurrent = m_LeftKick.getSupplyCurrent();
        kickerLeftTorqueCurrent = m_LeftKick.getTorqueCurrent();
        kickerLeftTempCelsius = m_LeftKick.getDeviceTemp();
        kickerRightPosition = m_RightKick.getPosition();
        kickerRightVelocity = m_RightKick.getVelocity();
        kickerRightAppliedVolts = m_RightKick.getMotorVoltage();
        kickerRightSupplyCurrent = m_RightKick.getSupplyCurrent();
        kickerRightTorqueCurrent = m_RightKick.getTorqueCurrent();
        kickerRightTempCelsius = m_RightKick.getDeviceTemp();
        FloorPosition = m_Floor.getPosition();
        FloorVelocity = m_Floor.getVelocity();
        FloorAppliedVolts = m_Floor.getMotorVoltage();
        FloorSupplyCurrent = m_Floor.getSupplyCurrent();
        FloorTorqueCurrent = m_Floor.getTorqueCurrent();
        FloorTempCelsius = m_Floor.getDeviceTemp();
        LoadPosition = m_Loader.getPosition();
        LoadVelocity = m_Loader.getVelocity();
        LoadAppliedVolts = m_Loader.getMotorVoltage();
        LoadSupplyCurrent = m_Loader.getSupplyCurrent();
        LoadTorqueCurrent = m_Loader.getTorqueCurrent();
        LoadTempCelsius = m_Loader.getDeviceTemp();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100.0,
        kickerLeftPosition,
        kickerLeftVelocity,
        kickerLeftAppliedVolts,
        kickerLeftSupplyCurrent,
        kickerLeftTorqueCurrent,
        kickerLeftTempCelsius,
        kickerRightPosition,
        kickerRightVelocity,
        kickerRightAppliedVolts,
        kickerRightSupplyCurrent,
        kickerRightTorqueCurrent,
        kickerRightTempCelsius,
        FloorPosition,
        FloorVelocity,
        FloorAppliedVolts,
        FloorSupplyCurrent,
        FloorTorqueCurrent,
        FloorTempCelsius,
        LoadPosition,
        LoadVelocity,
        LoadAppliedVolts,
        LoadSupplyCurrent,
        LoadTorqueCurrent,
        LoadTempCelsius
    );

    if(Utils.isSimulation()){
        PhysicsSim.getInstance().addTalonFX(m_LeftKick, 0.001);
        PhysicsSim.getInstance().addTalonFX(m_RightKick, 0.001);
        PhysicsSim.getInstance().addTalonFX(m_Floor, 0.001);
        PhysicsSim.getInstance().addTalonFX(m_Loader, 0.001);
    }

    }

    public void startIndex(double speed) {
        
        double indexerSpeed = 50.0; //IMPORTANT
        double feederSpeed = 50.0; //IMPORTANT
        
        switch (controlMode) {
            case 0:
            m_LeftKick.setControl(m_VelocityVoltage.withVelocity(indexerSpeed));
            m_RightKick.setControl(m_VelocityVoltage.withVelocity(indexerSpeed));
            m_Floor.setControl(m_VelocityVoltage.withVelocity(feederSpeed));
            m_Loader.setControl(m_VelocityVoltage.withVelocity(feederSpeed));
        }
    }

    

    public void stopIndex(double speed) {
        double desiredRotationsPerSecond = 0; //IMPORTANT
        switch (controlMode) {
            case 0:
            m_Loader.setControl(m_VelocityVoltage.withVelocity(desiredRotationsPerSecond));
            m_Floor.setControl(m_VelocityVoltage.withVelocity(desiredRotationsPerSecond));
            m_RightKick.setControl(m_VelocityVoltage.withVelocity(desiredRotationsPerSecond));
            m_LeftKick.setControl(m_VelocityVoltage.withVelocity(desiredRotationsPerSecond));
        }
}

public void stop() {
    m_LeftKick.setControl(m_brake);
    m_RightKick.setControl(m_brake);
    m_Floor.setControl(m_brake);
    m_Loader.setControl(m_brake);
}
}
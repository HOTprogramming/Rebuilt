package frc.robot.subsystems.Hood;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import frc.robot.Constants;

public class HoodConstants {
    public static int kHoodMotorID;
    public static int kHoodEncoderID;
    public static double hoodPosition0 = 0.0;
    public static double hoodPosition1 = 15.0;
    public static double hoodPosition2 = 30.0;
    public static double hoodPosition3 = 45.0;
    public static double hoodPosition4 = 60.0;
    public static double kReduction = (1.0/1.0);
    public static CANBus canbus = new CANBus("rio");
    public static double kEncoderOffset = -0.072266;

    // Define configuration for Intake Roller Motor 
    public static final TalonFXConfiguration hoodMotorConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(40)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(30)
                .withSupplyCurrentLowerTime(1)
        ).withTorqueCurrent(
            new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(120)
                .withPeakReverseTorqueCurrent(-120)
        ).withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)    
                .withNeutralMode(NeutralModeValue.Coast)   
        ).withSlot0(
            new Slot0Configs()            /* Voltage-based position */
                .withKP(2.4)        // An error of 1 rotation results in 2.4v output
                .withKI(0.0)        // No output for integrated error
                .withKD(0.1)        // A velocity of 1 rps results in 0.1v output 
        ).withSlot1(
            new Slot1Configs()            /* Torque-based position  */
                .withKP(60.0)       // An error of 1 rotation results in 60 A output
                .withKI(0.0)        // No output for integrated error     
                .withKD(6.0)        // A velocity of 1 rps results in 6 A output 
        );

    /********************************************************
    * Based on the robot selected, load Hood configs with
    *   the selected robot configuration  
    ********************************************************/
    public static final HoodConfigs HoodConfigs =
        switch (Constants.getRobot()) {
            case COMPBOT -> new HoodConfigs(
                hoodMotorConfigs
            );
            case SIMBOT -> new HoodConfigs(
                hoodMotorConfigs
            );
            case DEVBOT -> new HoodConfigs(
                hoodMotorConfigs
            );
            case CAMERABOT -> new HoodConfigs( 
                hoodMotorConfigs
            );
            case COMP24 -> new HoodConfigs( 
                hoodMotorConfigs
            );
            case COMP25 -> new HoodConfigs( 
                hoodMotorConfigs
            );
            case MINIBOT -> new HoodConfigs( 
                hoodMotorConfigs
            );
        };

    /*****************
     * Define the Hood Config Class 
     *****************/
    public record HoodConfigs(
        TalonFXConfiguration hoodMotorConfigs
    ) {}

}

package frc.robot.subsystems.Shooter;

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

public class ShooterConstants {
    public static int kMotorID = 15;
    public static int kOtherMotorID = 14;
    public static double kReduction =(1.0/1.0);
    public static double kIdleSpeed = (10.0);
    public static double kShooterSpeed = (40.0);
    public static CANBus canbus = new CANBus("rio");
    public static double kEncoderOffset = -0.072266;

    // Define configuration for Shooter Motor 1 //
    public static final TalonFXConfiguration shooterMotorConfigs = new TalonFXConfiguration()
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
                .withPeakForwardTorqueCurrent(40)
                .withPeakReverseTorqueCurrent(-40)
        ).withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.CounterClockwise_Positive)    
                .withNeutralMode(NeutralModeValue.Coast)   
        ).withSlot0(
            new Slot0Configs()            /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
                .withKP(0.1)       // An error of 1 rotation per second results in 0.11 V output
                .withKI(0.0)        // No output for integrated error
                .withKD(0.0)        // No output for error derivative
                .withKS(0.19)        // To account for friction, add 0.1 V of static feedforward
                .withKV(0.11)       // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        ).withSlot1(
            new Slot1Configs()            /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
                .withKP(5.0)        // An error of 1 rotation per second results in 5 A output
                .withKI(0.0)        // No output for integrated error
                .withKD(0.0)        // No output for error derivative
                .withKS(2.5)        // To account for friction, add 2.5 A of static feedforward        
        );

    // Define configuration for Shooter Motor 2 //
    public static final TalonFXConfiguration shooterOtherMotorConfigs = new TalonFXConfiguration()
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
                .withPeakForwardTorqueCurrent(40)
                .withPeakReverseTorqueCurrent(-40)
        ).withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)    
                .withNeutralMode(NeutralModeValue.Coast)   
        ).withSlot0(
            new Slot0Configs()            /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
                .withKP(0.1)       // An error of 1 rotation per second results in 0.11 V output
                .withKI(0.0)        // No output for integrated error
                .withKD(0.0)        // No output for error derivative
                .withKS(0.19)        // To account for friction, add 0.1 V of static feedforward
                .withKV(0.11)       // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        ).withSlot1(
            new Slot1Configs()            /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
                .withKP(5.0)        // An error of 1 rotation per second results in 5 A output
                .withKI(0.0)        // No output for integrated error
                .withKD(0.0)        // No output for error derivative
                .withKS(2.5)        // To account for friction, add 2.5 A of static feedforward        
        );


    /********************************************************
    * Based on the robot selected, load Shooter configs with
    *   the selected robot configuration  
    ********************************************************/
    public static final ShooterConfigs ShooterConfigs =
        switch (Constants.getRobot()) {
            case COMPBOT -> new ShooterConfigs(
                shooterMotorConfigs, shooterOtherMotorConfigs
            );
            case SIMBOT -> new ShooterConfigs(
                shooterMotorConfigs, shooterOtherMotorConfigs 
            );
            case DEVBOT -> new ShooterConfigs(
                shooterMotorConfigs, shooterOtherMotorConfigs
            );
            case CAMERABOT -> new ShooterConfigs( 
                shooterMotorConfigs, shooterOtherMotorConfigs
            );
            case COMP24 -> new ShooterConfigs( 
                shooterMotorConfigs, shooterOtherMotorConfigs
            );
            case COMP25 -> new ShooterConfigs( 
                shooterMotorConfigs, shooterOtherMotorConfigs
            );
            case MINIBOT -> new ShooterConfigs( 
                shooterMotorConfigs, shooterOtherMotorConfigs
            );
        };


    /*****************
     * Define the Shooter Config Class 
     *****************/
    public record ShooterConfigs(
        TalonFXConfiguration shooterMotorConfigs, TalonFXConfiguration shooterOtherMotorConfigs
    ) {}

}
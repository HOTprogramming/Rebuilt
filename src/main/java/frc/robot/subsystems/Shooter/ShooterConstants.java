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
    public static int kMotorID = 10;
    public static int kSub1MotorID = 11;
    public static int kSub2MotorID = 0;
    public static int kSub3MotorID = 0;
    public static double kShootReduction =(1.0/1.0);
    public static double kIdleSpeed = (70.0);
    public static double kShooterSpeed = (32.0);
    public static double kTargetShootSpeed = kShooterSpeed;
    public static double kShooterError = (0.500);

    public static double kTargetHoodAngle = 0;
    public static int kHoodMotorID = 12;
    public static double kHoodReduction = (1.0/1.0);

    public static CANBus canbus = new CANBus(Constants.canName);

    // Define configuration for Shooter Motor 1 //
    public static final TalonFXConfiguration shooterMotorConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(120)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(80)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(30)
                .withSupplyCurrentLowerTime(1)
        ).withTorqueCurrent(
            new TorqueCurrentConfigs()
            
            .withPeakForwardTorqueCurrent(80)
                .withPeakReverseTorqueCurrent(-80)
        ).withMotorOutput(
            new MotorOutputConfigs()
                .withInverted(InvertedValue.Clockwise_Positive)    
                .withNeutralMode(NeutralModeValue.Coast)   
        ).withSlot0(
            new Slot0Configs()            /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
                .withKP(2)       // An error of 1 rotation per second results in 0.11 V output
                .withKI(0.0)        // No output for integrated error
                .withKD(0.05)        // No output for error derivative
                .withKS(0.1)        // To account for friction, add 0.1 V of static feedforward
                .withKV(0.3)       // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        ).withSlot1(
            new Slot1Configs()            /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
                .withKP(19)        // An error of 1 rotation per second results in 5 A output
                .withKI(0.0)        // No output for integrated error
                .withKD(0.0)        // No output for error derivative
                .withKS(12)
                .withKV(.68)        // To account for friction, add 2.5 A of static feedforward        
        );

    //Define configuration for Hood Motor//
    public static final TalonFXConfiguration hoodMotorConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(80)
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
                .withKP(2.5)        // An error of 1 rotation results in 2.4v output
                .withKI(0.0)        // No output for integrated error
                .withKD(0)        // A velocity of 1 rps results in 0.1v output 
                .withKS(.1)
                .withKV(.1)
        ).withSlot1(
            new Slot1Configs()            /* Torque-based position  */
                .withKP(60.0)       // An error of 1 rotation results in 60 A output
                .withKI(0.0)        // No output for integrated error     
                .withKD(6.0)        // A velocity of 1 rps results in 6 A output 
        );


    /********************************************************
    * Based on the robot selected, load Shooter configs with
    *   the selected robot configuration  
    ********************************************************/
    public static final ShooterConfigs ShooterConfigs =
        switch (Constants.getRobot()) {
            case COMPBOT -> new ShooterConfigs(
                shooterMotorConfigs, hoodMotorConfigs
            );
            case SIMBOT -> new ShooterConfigs(
                shooterMotorConfigs, hoodMotorConfigs 
            );
            case DEVBOT -> new ShooterConfigs(
                shooterMotorConfigs, hoodMotorConfigs
            );
            case CAMERABOT -> new ShooterConfigs( 
                shooterMotorConfigs, hoodMotorConfigs
            );
            case COMP24 -> new ShooterConfigs( 
                shooterMotorConfigs, hoodMotorConfigs
            );
            case COMP25 -> new ShooterConfigs( 
                shooterMotorConfigs, hoodMotorConfigs
            );
            case MINIBOT -> new ShooterConfigs( 
                shooterMotorConfigs, hoodMotorConfigs
            );
            case ALPHABOT -> new ShooterConfigs( 
                shooterMotorConfigs, hoodMotorConfigs
            );
        };


    /*****************
     * Define the Shooter Config Class 
     *****************/
    public record ShooterConfigs(
        TalonFXConfiguration shooterMotorConfigs, TalonFXConfiguration hoodMotorConfigs
    ) {}

}
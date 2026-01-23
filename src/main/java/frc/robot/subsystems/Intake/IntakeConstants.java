package frc.robot.subsystems.Intake;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.Slot1Configs;
import com.ctre.phoenix6.configs.Slot2Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import frc.robot.Constants;

public class IntakeConstants {
    public static double kLeftDeployedPosition = 200.0;
    public static double kRightDeployedPosition = 200.0;
    public static double kLeftRetractPosition = 0.0;
    public static double kRightRetractPosition = 0.0;

    public static double kLeftExpel = -1000.0;
    public static double kRightExpel = -1000.0;

    public static int kLeftIntakeEncoderID = 20;
    public static int kRightIntakeEncoderID = 21;

    public static double kLeftIntake = 2000.0;
    public static double kRightIntake = 2000.0;

    public static int kIntakeMotorIDLExtend = 15;
    public static int kIntakeMotorIDRExtend = 16;
    public static int kIntakeMotorIDLRoller = 17;
    public static int kIntakeMotorIDRRoller = 18;
    public static double kEncoderOffset = -0.072266;
    public static String kBusName = "sim";

    public static final TalonFXConfiguration LeftRollerConfigs = new TalonFXConfiguration()
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
        ).withMotionMagic(
            new MotionMagicConfigs() 
                .withMotionMagicCruiseVelocity(10.0)
                .withMotionMagicAcceleration(10.0)
        ).withSlot0(
            new Slot0Configs()            /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
                .withKP(0.11)       // An error of 1 rotation per second results in 0.11 V output
                .withKI(0.0)        // No output for integrated error
                .withKD(0.0)        // No output for error derivative
                .withKS(0.1)        // To account for friction, add 0.1 V of static feedforward
                .withKV(0.12)       // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        ).withSlot1(
            new Slot1Configs()            /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
                .withKP(5.0)        // An error of 1 rotation per second results in 5 A output
                .withKI(0.0)        // No output for integrated error
                .withKD(0.0)        // No output for error derivative
                .withKS(2.5)        // To account for friction, add 2.5 A of static feedforward        
        ).withSlot2(                  
            new Slot2Configs()            /* Motion Magic Velocity  */  
                .withKS(0.25)       // Add 0.25 V output to overcome static friction
                .withKV(.12)        // A velocity target of 1 rps results in 0.12 V output
                .withKA(0.01)       // An acceleration of 1 rps/s requires 0.01 V output
                .withKP(4.8)        // A position error of 2.5 rotations results in 12 V output
                .withKI(0)          // no output for integrated error
                .withKD(0.1)        // A velocity error of 1 rps results in 0.1 V output
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(80.0)   // Target cruise velocity of 80 rps
                .withMotionMagicAcceleration(160)      // Target acceleration of 160 rps/s (0.5 seconds)
                .withMotionMagicJerk(1600)               // Target jerk of 1600 rps/s/s (0.1 seconds))    

            );

            public static final TalonFXConfiguration RightRollerConfigs = new TalonFXConfiguration()
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
        ).withMotionMagic(
            new MotionMagicConfigs() 
                .withMotionMagicCruiseVelocity(10.0)
                .withMotionMagicAcceleration(10.0)
        ).withSlot0(
            new Slot0Configs()            /* Voltage-based velocity requires a velocity feed forward to account for the back-emf of the motor */
                .withKP(0.11)       // An error of 1 rotation per second results in 0.11 V output
                .withKI(0.0)        // No output for integrated error
                .withKD(0.0)        // No output for error derivative
                .withKS(0.1)        // To account for friction, add 0.1 V of static feedforward
                .withKV(0.12)       // Kraken X60 is a 500 kV motor, 500 rpm per V = 8.333 rps per V, 1/8.33 = 0.12 volts / rotation per second
        ).withSlot1(
            new Slot1Configs()            /* Torque-based velocity does not require a velocity feed forward, as torque will accelerate the rotor up to the desired velocity by itself */
                .withKP(5.0)        // An error of 1 rotation per second results in 5 A output
                .withKI(0.0)        // No output for integrated error
                .withKD(0.0)        // No output for error derivative
                .withKS(2.5)        // To account for friction, add 2.5 A of static feedforward        
        ).withSlot2(                  
            new Slot2Configs()            /* Motion Magic Velocity  */  
                .withKS(0.25)       // Add 0.25 V output to overcome static friction
                .withKV(.12)        // A velocity target of 1 rps results in 0.12 V output
                .withKA(0.01)       // An acceleration of 1 rps/s requires 0.01 V output
                .withKP(4.8)        // A position error of 2.5 rotations results in 12 V output
                .withKI(0)          // no output for integrated error
                .withKD(0.1)        // A velocity error of 1 rps results in 0.1 V output
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(80.0)   // Target cruise velocity of 80 rps
                .withMotionMagicAcceleration(160)      // Target acceleration of 160 rps/s (0.5 seconds)
                .withMotionMagicJerk(1600)               // Target jerk of 1600 rps/s/s (0.1 seconds))    

            );

            public static final TalonFXConfiguration LeftIntakeExtendConfigs = new TalonFXConfiguration()
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
        ).withSlot2(                  
            new Slot2Configs()            /* Motion Magic Expo  */  
                .withKS(0.25)       // Add 0.25 V output to overcome static friction
                .withKV(.12)        // A velocity target of 1 rps results in 0.12 V output
                .withKA(0.01)       // An acceleration of 1 rps/s requires 0.01 V output
                .withKP(4.8)        // A position error of 2.5 rotations results in 12 V output
                .withKI(0)          // no output for integrated error
                .withKD(0.1)        // A velocity error of 1 rps results in 0.1 V output
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(0.0)   // Unlimited cruise velocity
                .withMotionMagicExpo_kV(0.12)          // kV is around 0.12 V/rps
                .withMotionMagicExpo_kA(0.1)          // Use a slower kA of 0.1 V/(rps/s)
         ).withFeedback(
            new FeedbackConfigs() 
                .withFeedbackRemoteSensorID(kLeftIntakeEncoderID)        
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder) 
                .withSensorToMechanismRatio(1.0)  //The ratio of sensor rotations to the mechanism's output, where a ratio greater than 1 is a reduction.
                .withRotorToSensorRatio(1.0)   //The ratio of motor rotor rotations to remote sensor rotations, where a ratio greater than 1 is a reduction.
            );

            public static final TalonFXConfiguration RightIntakeExtendConfigs = new TalonFXConfiguration()
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
        ).withSlot2(                  
            new Slot2Configs()            /* Motion Magic Expo  */  
                .withKS(0.25)       // Add 0.25 V output to overcome static friction
                .withKV(.12)        // A velocity target of 1 rps results in 0.12 V output
                .withKA(0.01)       // An acceleration of 1 rps/s requires 0.01 V output
                .withKP(4.8)        // A position error of 2.5 rotations results in 12 V output
                .withKI(0)          // no output for integrated error
                .withKD(0.1)        // A velocity error of 1 rps results in 0.1 V output
        ).withMotionMagic(
            new MotionMagicConfigs()
                .withMotionMagicCruiseVelocity(0.0)   // Unlimited cruise velocity
                .withMotionMagicExpo_kV(0.12)          // kV is around 0.12 V/rps
                .withMotionMagicExpo_kA(0.1)          // Use a slower kA of 0.1 V/(rps/s)
        ).withFeedback(
            new FeedbackConfigs() 
                .withFeedbackRemoteSensorID(kRightIntakeEncoderID)        
                .withFeedbackSensorSource(FeedbackSensorSourceValue.FusedCANcoder) 
                .withSensorToMechanismRatio(1.0)  //The ratio of sensor rotations to the mechanism's output, where a ratio greater than 1 is a reduction.
                .withRotorToSensorRatio(1.0)   //The ratio of motor rotor rotations to remote sensor rotations, where a ratio greater than 1 is a reduction.
            );

        public static final CANcoderConfiguration LeftIntakeEncoderConfigs = new CANcoderConfiguration() 
        .withMagnetSensor(
            new MagnetSensorConfigs()
                .withMagnetOffset(kEncoderOffset)    //The offset applied to the absolute integrated rotor sensor.
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        );
        public static final CANcoderConfiguration RightIntakeEncoderConfigs = new CANcoderConfiguration() 
        .withMagnetSensor(
            new MagnetSensorConfigs()
                .withMagnetOffset(kEncoderOffset)    //The offset applied to the absolute integrated rotor sensor.
                .withSensorDirection(SensorDirectionValue.CounterClockwise_Positive)
        );
    
             public static final IntakeConfigs IntakeConfigs =
        switch (Constants.getRobot()) {
            case COMPBOT -> new IntakeConfigs(
             LeftRollerConfigs,
             RightRollerConfigs,
             LeftIntakeExtendConfigs,
             RightIntakeExtendConfigs,
             RightIntakeEncoderConfigs,
             LeftIntakeEncoderConfigs
            );
            case SIMBOT -> new IntakeConfigs(
             LeftRollerConfigs,
             RightRollerConfigs,
             LeftIntakeExtendConfigs,
             RightIntakeExtendConfigs,
             RightIntakeEncoderConfigs,
             LeftIntakeEncoderConfigs
            );
            case DEVBOT -> new IntakeConfigs(
             LeftRollerConfigs,
             RightRollerConfigs,
             LeftIntakeExtendConfigs,
             RightIntakeExtendConfigs,
             RightIntakeEncoderConfigs,
             LeftIntakeEncoderConfigs
            );
            case CAMERABOT -> new IntakeConfigs( 
             LeftRollerConfigs,
             RightRollerConfigs,
             LeftIntakeExtendConfigs,
             RightIntakeExtendConfigs,
             RightIntakeEncoderConfigs,
             LeftIntakeEncoderConfigs
            );
            case COMP24 -> new IntakeConfigs(
             LeftRollerConfigs,
             RightRollerConfigs,
             LeftIntakeExtendConfigs,
             RightIntakeExtendConfigs,
             RightIntakeEncoderConfigs,
             LeftIntakeEncoderConfigs
            );
            case COMP25 -> new IntakeConfigs(
             LeftRollerConfigs,
             RightRollerConfigs,
             LeftIntakeExtendConfigs,
             RightIntakeExtendConfigs,
             RightIntakeEncoderConfigs,
             LeftIntakeEncoderConfigs
            );
            case MINIBOT -> new IntakeConfigs(
             LeftRollerConfigs,
             RightRollerConfigs,
             LeftIntakeExtendConfigs,
             RightIntakeExtendConfigs,
             RightIntakeEncoderConfigs,
             LeftIntakeEncoderConfigs
            );
        };

        public record IntakeConfigs(
            TalonFXConfiguration LeftRollerConfigs,
            TalonFXConfiguration RightRollerConfigs,
            TalonFXConfiguration LeftIntakeExtendConfigs,
            TalonFXConfiguration RightIntakeExtendConfigs,
            CANcoderConfiguration RightIntakeEncoderConfigs,
            CANcoderConfiguration LeftIntakeEncoderConfigs 
        ) {}
}

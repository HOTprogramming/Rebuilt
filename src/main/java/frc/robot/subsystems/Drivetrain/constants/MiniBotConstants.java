package frc.robot.subsystems.Drivetrain.constants;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;

import edu.wpi.first.units.measure.*;

import frc.robot.subsystems.Drivetrain.DriveConstants;

public final class MiniBotConstants {

    // Gear ratios
    private static final double kCoupleRatio = 2.0;
    private static final double kDriveGearRatio = 6.545454545454545;
    private static final double kSteerGearRatio = 10.285714285714286;

    // CAN bus
    public static final CANBus kCANBus = new CANBus("drivetrain", "./logs/example.hoot");
    private static final int kPigeonId = 50;

    // Drivetrain constants
    public static final SwerveDrivetrainConstants DrivetrainConstants = new SwerveDrivetrainConstants()
        .withCANBusName(kCANBus.getName())
        .withPigeon2Id(kPigeonId)
        .withPigeon2Configs(null);

    // Friction voltages
    private static final double kSteerFrictionVoltage = 0.2;
    private static final double kDriveFrictionVoltage = 0.2;

    // Slip current
    private static final double kSlipCurrent = 300.0;

    // Wheel, inversion, and inertia
    private static final Distance kWheelRadius = Inches.of(2);
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);

    // Speed and motor types
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.58);
    public static final AngularVelocity kMaxAngularVelocity = RotationsPerSecond.of(2);
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // Steer gains
    private static final Slot0Configs steerGains = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(1.28).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    // Drive gains
    private static final Slot0Configs driveGains = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0.0).withKV(0.124).withKA(0.0);

    // Closed-loop output types
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;

    // Module factory
    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreator =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatio)
            .withSteerMotorGearRatio(kSteerGearRatio)
            .withCouplingGearRatio(kCoupleRatio)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGains)
            .withDriveMotorGains(driveGains)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(DriveConstants.driveInitialConfigs)
            .withSteerMotorInitialConfigs(DriveConstants.steerInitialConfigs)
            .withEncoderInitialConfigs(DriveConstants.encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Front Left
    private static final int kFrontLeftDriveMotorId = 6;
    private static final int kFrontLeftSteerMotorId = 5;
    private static final int kFrontLeftEncoderId = 42;
    private static final Angle kFrontLeftEncoderOffset = Rotations.of(-0.180908203125);
    private static final boolean kFrontLeftSteerMotorInverted = true;
    private static final boolean kFrontLeftEncoderInverted = false;
    private static final Distance kFrontLeftXPos = Inches.of(9);
    private static final Distance kFrontLeftYPos = Inches.of(9);

    // Front Right
    private static final int kFrontRightDriveMotorId = 8;
    private static final int kFrontRightSteerMotorId = 7;
    private static final int kFrontRightEncoderId = 44;
    private static final Angle kFrontRightEncoderOffset = Rotations.of(-0.12255859375);
    private static final boolean kFrontRightSteerMotorInverted = true;
    private static final boolean kFrontRightEncoderInverted = false;
    private static final Distance kFrontRightXPos = Inches.of(9);
    private static final Distance kFrontRightYPos = Inches.of(-9);

    // Back Left
    private static final int kBackLeftDriveMotorId = 4;
    private static final int kBackLeftSteerMotorId = 3;
    private static final int kBackLeftEncoderId = 41;
    private static final Angle kBackLeftEncoderOffset = Rotations.of(0.168212890625);
    private static final boolean kBackLeftSteerMotorInverted = true;
    private static final boolean kBackLeftEncoderInverted = false;
    private static final Distance kBackLeftXPos = Inches.of(-9);
    private static final Distance kBackLeftYPos = Inches.of(9);

    // Back Right
    private static final int kBackRightDriveMotorId = 2;
    private static final int kBackRightSteerMotorId = 1;
    private static final int kBackRightEncoderId = 43;
    private static final Angle kBackRightEncoderOffset = Rotations.of(0.279541015625);
    private static final boolean kBackRightSteerMotorInverted = true;
    private static final boolean kBackRightEncoderInverted = false;
    private static final Distance kBackRightXPos = Inches.of(-9);
    private static final Distance kBackRightYPos = Inches.of(-9);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeft =
        ConstantCreator.createModuleConstants(
            kFrontLeftSteerMotorId, kFrontLeftDriveMotorId, kFrontLeftEncoderId, kFrontLeftEncoderOffset,
            kFrontLeftXPos, kFrontLeftYPos, kInvertLeftSide, kFrontLeftSteerMotorInverted, kFrontLeftEncoderInverted
        ).withDriveMotorInverted(false);

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRight =
        ConstantCreator.createModuleConstants(
            kFrontRightSteerMotorId, kFrontRightDriveMotorId, kFrontRightEncoderId, kFrontRightEncoderOffset,
            kFrontRightXPos, kFrontRightYPos, kInvertRightSide, kFrontRightSteerMotorInverted, kFrontRightEncoderInverted
        );

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeft =
        ConstantCreator.createModuleConstants(
            kBackLeftSteerMotorId, kBackLeftDriveMotorId, kBackLeftEncoderId, kBackLeftEncoderOffset,
            kBackLeftXPos, kBackLeftYPos, kInvertLeftSide, kBackLeftSteerMotorInverted, kBackLeftEncoderInverted
        );

    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRight =
        ConstantCreator.createModuleConstants(
            kBackRightSteerMotorId, kBackRightDriveMotorId, kBackRightEncoderId, kBackRightEncoderOffset,
            kBackRightXPos, kBackRightYPos, kInvertRightSide, kBackRightSteerMotorInverted, kBackRightEncoderInverted
        );
}

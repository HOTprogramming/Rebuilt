package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;

import java.util.HashMap;
import java.util.Map;

import com.ctre.phoenix6.CANBus;
import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import com.ctre.phoenix6.swerve.*;
import com.ctre.phoenix6.swerve.SwerveModuleConstants.*;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.*;

import frc.robot.Constants;



public class DriveConstants {

    public static final double slowModeMultiplier = 0.5;
    public static final double kTranslationRateLimiter = 4; // unitless per second^2, this is from the joystick 0-1 command
    public static final double kRotationRateLimiter = 4; // unitless per second^2, this is from the joystick 0-1 command
    public static final double OTF_end_tolerance = 0.2;

    public static final double auto_align_theta_disable = .1;
    public static final double auto_align_top_speed_teleop = 2.4;
    public static final double auto_align_slow_speed_teleop = 1.0;
    public static final double auto_align_top_speed_auton = 2.4;
    public static final double auto_align_slow_in_percent = 0.66;
    public static final double auto_align_tolerance = 0.01;
    public static final double auto_align_lights_tolerance = Units.inchesToMeters(1.5);
    public static final double auto_align_command = 0.035;

    public static final double distance_safe_from_reef = 1.1;
    public static final double feild_center_line = 4.025;

    private static final PIDConstants autoPidConstantsTranslation = new PIDConstants(5, 0, 0);
    private static final PIDConstants autoPidConstantsTheta = new PIDConstants (7, 0, 0); 
    private static final PIDConstants teleopPidConstantsTheta = new PIDConstants (10, 0, 0.5); 
    // Theoretical free speed (m/s) at 12 V applied output;
    // This needs to be tuned to your individual robot
    public static final AngularVelocity kMaxAngularVelocity = RotationsPerSecond.of(2);
    public static final LinearVelocity kSpeedAt12Volts = MetersPerSecond.of(4.58);


    public static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS = new TrapezoidProfile.Constraints(
        1.6,
        kSpeedAt12Volts.in(MetersPerSecond) * .7);

    public static final Map<Rotation2d, Pose2d> redPoses = new HashMap<>() {
        {
            put(Rotation2d.fromDegrees(0), new Pose2d(12.23, 4.03, Rotation2d.fromDegrees(0)));
            put(Rotation2d.fromDegrees(60), new Pose2d(12.64, 3.31, Rotation2d.fromDegrees(60)));
            put(Rotation2d.fromDegrees(120), new Pose2d(13.47, 3.31, Rotation2d.fromDegrees(120)));
            put(Rotation2d.fromDegrees(180), new Pose2d(13.89, 4.03, Rotation2d.fromDegrees(180)));
            put(Rotation2d.fromDegrees(-120), new Pose2d(13.47, 4.75, Rotation2d.fromDegrees(-120)));
            put(Rotation2d.fromDegrees(-60), new Pose2d(12.64, 4.75, Rotation2d.fromDegrees(-60))); 
        }
    };

    public static final Map<Rotation2d, Pose2d> bluePoses = new HashMap<>() {
        {
            put(Rotation2d.fromDegrees(0), new Pose2d(3.66, 4.03, Rotation2d.fromDegrees(0)));
            put(Rotation2d.fromDegrees(60), new Pose2d(4.07, 3.31, Rotation2d.fromDegrees(60)));
            put(Rotation2d.fromDegrees(120), new Pose2d(4.9, 3.31, Rotation2d.fromDegrees(120)));
            put(Rotation2d.fromDegrees(180), new Pose2d(5.32, 4.03, Rotation2d.fromDegrees(180)));
            put(Rotation2d.fromDegrees(-120), new Pose2d(4.90, 4.75, Rotation2d.fromDegrees(-120)));
            put(Rotation2d.fromDegrees(-60), new Pose2d(4.07,4.75, Rotation2d.fromDegrees(-60))); 
        }
    };
    // //                                  COMP GAINS
    // public static final Map<Rotation2d, Double[]> redPoleShift = new HashMap<>() {
    //     {   //                                                  left center right
    //         put(Rotation2d.fromDegrees(0), new Double[] {6.0, -1.6, -6.0});
    //         put(Rotation2d.fromDegrees(60), new Double[] {6.0, -1.6,  -6.0});
    //         put(Rotation2d.fromDegrees(120), new Double[] {6.0, -1.6,  -6.0});
    //         put(Rotation2d.fromDegrees(180), new Double[] {6.2, -1.6, -6.5});
    //         put(Rotation2d.fromDegrees(-120), new Double[] {6.0, -1.6, -6.0});
    //         put(Rotation2d.fromDegrees(-60), new Double[] {6.0, -1.6,  -6.0});
    //     }
    // };

    // public static final Map<Rotation2d, Double[]> bluePoleShift = new HashMap<>() {
    //     {   //                                                  left center right
    //         put(Rotation2d.fromDegrees(0), new Double[] {6.0, -0.25, -6.0}); // left maybe -
    //         put(Rotation2d.fromDegrees(60), new Double[] {6.0, -0.25,  -6.5});
    //         put(Rotation2d.fromDegrees(120), new Double[] {6.0, -0.25, -6.2});
    //         put(Rotation2d.fromDegrees(180), new Double[] {6.0, -0.25,  -6.0});
    //         put(Rotation2d.fromDegrees(-120), new Double[] {6.0, -0.25, -6.3});
    //         put(Rotation2d.fromDegrees(-60), new Double[] {6.5, -0.25, -6.0});
    //     }
    // };

    public static final double robotToReefTagFace = 0.39;

    //                             GM GAINS
    public static final Map<Rotation2d, Double[]> redPoleShift = new HashMap<>() {
        {   //                                                  left center right
            put(Rotation2d.fromDegrees(0), new Double[] {6.75, -0.25, -6.0});
            put(Rotation2d.fromDegrees(60), new Double[] {8.0, -0.25,  -7.0});
            put(Rotation2d.fromDegrees(120), new Double[] {5.0, -0.25, -8.0});
            put(Rotation2d.fromDegrees(180), new Double[] {6.0, -0.25, -7.5});
            put(Rotation2d.fromDegrees(-120), new Double[] {6.5, -0.25, -7.0});
            put(Rotation2d.fromDegrees(-60), new Double[] {5.5, -1.25, -8.0});
        }
    };

    public static final Map<Rotation2d, Double[]> bluePoleShift = new HashMap<>() {
        {   //                                                  left center right
            put(Rotation2d.fromDegrees(0), new Double[] {6.0, -0.25, -7.5});
            put(Rotation2d.fromDegrees(60), new Double[] {6.25, -0.25, -8.0});
            put(Rotation2d.fromDegrees(120), new Double[] {5.5, -0.25, -8.0});
            put(Rotation2d.fromDegrees(180), new Double[] {7.0, -0.25, -6.0});
            put(Rotation2d.fromDegrees(-120), new Double[] {8.5, -0.25,  -7.0});
            put(Rotation2d.fromDegrees(-60), new Double[] {5.0, -0.25, -8.0});
        }
    };

    public static final Pose2d BLUE_REEF = new Pose2d(4.483, 4.025, Rotation2d.fromDegrees(0));
    public static final Pose2d RED_REEF = new Pose2d(13.066, 4.025, Rotation2d.fromDegrees(0));
    public static final double OFFSET_TO_RED = 8.583;
    public static final Pose2d REEF_CENTER = new Pose2d(4.483, 4.025, Rotation2d.fromDegrees(0));


    // The closed-loop output type to use for the steer motors;
    // This affects the PID/FF gains for the steer motors
    private static final ClosedLoopOutputType kSteerClosedLoopOutput = ClosedLoopOutputType.Voltage;
    // The closed-loop output type to use for the drive motors;
    // This affects the PID/FF gains for the drive motors
    private static final ClosedLoopOutputType kDriveClosedLoopOutput = ClosedLoopOutputType.TorqueCurrentFOC;
    private static final ClosedLoopOutputType kDriveClosedLoopOutputCambot = ClosedLoopOutputType.Voltage;

    // The type of motor used for the drive motor
    private static final DriveMotorArrangement kDriveMotorType = DriveMotorArrangement.TalonFX_Integrated;
    // The type of motor used for the drive motor
    private static final SteerMotorArrangement kSteerMotorType = SteerMotorArrangement.TalonFX_Integrated;

    // The remote sensor feedback type to use for the steer motors;
    // When not Pro-licensed, FusedCANcoder/SyncCANcoder automatically fall back to RemoteCANcoder
    private static final SteerFeedbackType kSteerFeedbackType = SteerFeedbackType.FusedCANcoder;

    // The stator current at which the wheels start to slip;
    // This needs to be tuned to your individual robot
    private static final Current kSlipCurrent = Amps.of(120.0);

    // Initial configs for the drive and steer motors and the azimuth encoder; these cannot be null.
    // Some configs will be overwritten; check the `with*InitialConfigs()` API documentation.
    public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(80)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(90)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(40)
                .withSupplyCurrentLowerTime(1)
        ).withTorqueCurrent(
            new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(70)
                .withPeakReverseTorqueCurrent(-70)
        );

    public static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(70)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(70)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(50)
                .withSupplyCurrentLowerTime(1)
        ).withTorqueCurrent(
            new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(120)
                .withPeakReverseTorqueCurrent(-120)
        );

    private static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();
    // Configs for the Pigeon 2; leave this null to skip applying Pigeon 2 configs
    private static final Pigeon2Configuration pigeonConfigs = null;

  

    private static final Distance kWheelRadius = Inches.of(2);
    private static final boolean kInvertLeftSide = false;
    private static final boolean kInvertRightSide = true;
    private static final int kPigeonId = 50;

    // These are only used for simulation
    private static final MomentOfInertia kSteerInertia = KilogramSquareMeters.of(0.01);
    private static final MomentOfInertia kDriveInertia = KilogramSquareMeters.of(0.01);
    
    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltage = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltage = Volts.of(0.2);

    /**************************************************************************************************
    * Define the MiniBot (Tiny Drive Only Robot) Swerve Constants  
    ***************************************************************************************************/
    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatioMinibot = 2;
    private static final double kDriveGearRatioMinibot = 6.545454545454545;
    private static final double kSteerGearRatioMinibot = 10.285714285714286;
  
    // CAN bus that the devices are located on; All swerve devices must share the same CAN bus
    public static final CANBus kCANBusMinibot = new CANBus("drivetrain", "./logs/example.hoot");

    public static final SwerveDrivetrainConstants DrivetrainConstantsMinibot = new SwerveDrivetrainConstants()
       .withCANBusName(kCANBusMinibot.getName())
       .withPigeon2Id(kPigeonId)
       .withPigeon2Configs(pigeonConfigs);

    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltageMinibot = 0.2;
    private static final double kDriveFrictionVoltageMinibot = 0.2;   
    private static final double kSlipCurrentMinibot = 300.0; // *tune later

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGainsMinibot = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(1.28).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
        ;
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGainsMinibot = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0.0).withKV(0.124).withKA(0.0);

    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPEMinibot = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPEMinibot = ClosedLoopOutputType.TorqueCurrentFOC;
    
    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorMinibot =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatioMinibot)
            .withSteerMotorGearRatio(kSteerGearRatioMinibot)
            .withCouplingGearRatio(kCoupleRatioMinibot)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGainsMinibot)
            .withDriveMotorGains(driveGainsMinibot)
            .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT_TYPEMinibot)
            .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT_TYPEMinibot)
            .withSlipCurrent(kSlipCurrentMinibot)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltageMinibot)
            .withDriveFrictionVoltage(kDriveFrictionVoltageMinibot);

    // Front Left
    private static final int kFrontLeftDriveMotorIdMinibot = 6;
    private static final int kFrontLeftSteerMotorIdMinibot = 5;
    private static final int kFrontLeftEncoderIdMinibot = 42;
    private static final Angle kFrontLeftEncoderOffsetMinibot = Rotations.of(-0.180908203125);
    private static final boolean kFrontLeftSteerMotorInvertedMinibot = true;
    private static final boolean kFrontLeftEncoderInvertedMinibot = false;  //?????????????????????????????????
    private static final Distance kFrontLeftXPosMinibot = Inches.of(9);
    private static final Distance kFrontLeftYPosMinibot = Inches.of(9);

    // Front Right
    private static final int kFrontRightDriveMotorIdMinibot = 8;
    private static final int kFrontRightSteerMotorIdMinibot = 7;
    private static final int kFrontRightEncoderIdMinibot = 44;
    private static final Angle kFrontRightEncoderOffsetMinibot = Rotations.of(-0.12255859375);
    private static final boolean kFrontRightSteerMotorInvertedMinibot = true;
    private static final boolean kFrontRightEncoderInvertedMinibot = false; //???????????????????????????????
    private static final Distance kFrontRightXPosMinibot = Inches.of(9);
    private static final Distance kFrontRightYPosMinibot = Inches.of(-9);

    // Back Left
    private static final int kBackLeftDriveMotorIdMinibot = 4;
    private static final int kBackLeftSteerMotorIdMinibot = 3;
    private static final int kBackLeftEncoderIdMinibot = 41;
    private static final Angle kBackLeftEncoderOffsetMinibot = Rotations.of(0.168212890625);
    private static final boolean kBackLeftSteerMotorInvertedMinibot = true;  
    private static final boolean kBackLeftEncoderInvertedMinibot = false;  //????????????????????????????????????
    private static final Distance kBackLeftXPosMinibot = Inches.of(-9);
    private static final Distance kBackLeftYPosMinibot = Inches.of(9);

    // Back Right
    private static final int kBackRightDriveMotorIdMinibot = 2;
    private static final int kBackRightSteerMotorIdMinibot = 1;
    private static final int kBackRightEncoderIdMinibot = 43;
    private static final Angle kBackRightEncoderOffsetMinibot = Rotations.of(0.279541015625);
    private static final boolean kBackRightSteerMotorInvertedMinibot = true;
    private static final boolean kBackRightEncoderInvertedMinibot = false; //??????????????????????????????????????

    private static final Distance kBackRightXPosMinibot = Inches.of(-9);
    private static final Distance kBackRightYPosMinibot = Inches.of(-9);


    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeftMinibot =
        ConstantCreatorMinibot.createModuleConstants(
            kFrontLeftSteerMotorIdMinibot, kFrontLeftDriveMotorIdMinibot, kFrontLeftEncoderIdMinibot, kFrontLeftEncoderOffsetMinibot,
            kFrontLeftXPosMinibot, kFrontLeftYPosMinibot, kInvertLeftSide, kFrontLeftSteerMotorInvertedMinibot, kFrontLeftEncoderInvertedMinibot
        ).withDriveMotorInverted(false);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRightMinibot =
        ConstantCreatorMinibot.createModuleConstants(
            kFrontRightSteerMotorIdMinibot, kFrontRightDriveMotorIdMinibot, kFrontRightEncoderIdMinibot, kFrontRightEncoderOffsetMinibot,
            kFrontRightXPosMinibot, kFrontRightYPosMinibot, kInvertRightSide, kFrontRightSteerMotorInvertedMinibot, kFrontRightEncoderInvertedMinibot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeftMinibot =
        ConstantCreatorMinibot.createModuleConstants(
            kBackLeftSteerMotorIdMinibot, kBackLeftDriveMotorIdMinibot, kBackLeftEncoderIdMinibot, kBackLeftEncoderOffsetMinibot,
            kBackLeftXPosMinibot, kBackLeftYPosMinibot, kInvertLeftSide, kBackLeftSteerMotorInvertedMinibot, kBackLeftEncoderInvertedMinibot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRightMinibot =
        ConstantCreatorMinibot.createModuleConstants(
            kBackRightSteerMotorIdMinibot, kBackRightDriveMotorIdMinibot, kBackRightEncoderIdMinibot, kBackRightEncoderOffsetMinibot,
            kBackRightXPosMinibot, kBackRightYPosMinibot, kInvertRightSide, kBackRightSteerMotorInvertedMinibot, kBackRightEncoderInvertedMinibot
        );


    /**************************************************************************************************
    * Define the Comp24 (2024 Comp Robot) Swerve Constants  
    ***************************************************************************************************/
    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatioComp24 = 3.5714285714285716;
    private static final double kDriveGearRatioComp24 = 5.01;
    private static final double kSteerGearRatioComp24 = 13.3714;
  
    // CAN bus that the devices are located on; All swerve devices must share the same CAN bus
    public static final CANBus kCANBusComp24 = new CANBus("drivetrain", "./logs/example.hoot");

    public static final SwerveDrivetrainConstants DrivetrainConstantsComp24 = new SwerveDrivetrainConstants()
       .withCANBusName(kCANBusComp24.getName())
       .withPigeon2Id(kPigeonId)
       .withPigeon2Configs(pigeonConfigs);

    // Simulated voltage necessary to overcome friction
    private static final double kSteerFrictionVoltageComp24 = 0.25;
    private static final double kDriveFrictionVoltageComp24 = 0.25;   
    private static final double kSlipCurrentComp24 = 300.0; // *tune later //500

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGainsComp24 = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.2)
        .withKS(0).withKV(1.5).withKA(0)
        //.withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
        ;
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGainsComp24 = new Slot0Configs()
        .withKP(3.0).withKI(0).withKD(0)
        .withKS(0.0).withKV(0.0).withKA(0.0);

    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPEComp24 = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPEComp24 = ClosedLoopOutputType.Voltage;
    
    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorComp24 =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatioComp24)
            .withSteerMotorGearRatio(kSteerGearRatioComp24)
            .withCouplingGearRatio(kCoupleRatioComp24)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGainsComp24)
            .withDriveMotorGains(driveGainsComp24)
            .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT_TYPEComp24)
            .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT_TYPEComp24)
            .withSlipCurrent(kSlipCurrentComp24)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltageComp24)
            .withDriveFrictionVoltage(kDriveFrictionVoltageComp24);

    // Front Left
    private static final int kFrontLeftDriveMotorIdComp24 = 7;
    private static final int kFrontLeftSteerMotorIdComp24 = 8;
    private static final int kFrontLeftEncoderIdComp24 = 40;
    private static final Angle kFrontLeftEncoderOffsetComp24 = Rotations.of(0.440673828125);
    private static final boolean kFrontLeftSteerMotorInvertedComp24 = false;
    private static final boolean kFrontLeftEncoderInvertedComp24 = true;  //?????????????????????????????????
    private static final Distance kFrontLeftXPosComp24 = Inches.of(10.125);
    private static final Distance kFrontLeftYPosComp24 = Inches.of(10.125);

    // Front Right
    private static final int kFrontRightDriveMotorIdComp24 = 3;
    private static final int kFrontRightSteerMotorIdComp24 = 4;
    private static final int kFrontRightEncoderIdComp24 = 41;
    private static final Angle kFrontRightEncoderOffsetComp24 = Rotations.of(0.098876953125);
    private static final boolean kFrontRightSteerMotorInvertedComp24 = false;
    private static final boolean kFrontRightEncoderInvertedComp24 = false; //???????????????????????????????
    private static final Distance kFrontRightXPosComp24 = Inches.of(10.125);
    private static final Distance kFrontRightYPosComp24 = Inches.of(-10.125);

    // Back Left
    private static final int kBackLeftDriveMotorIdComp24 = 5;
    private static final int kBackLeftSteerMotorIdComp24 = 6;
    private static final int kBackLeftEncoderIdComp24 = 42;
    private static final Angle kBackLeftEncoderOffsetComp24 = Rotations.of(-0.450439453125);
    private static final boolean kBackLeftSteerMotorInvertedComp24 = false;  
    private static final boolean kBackLeftEncoderInvertedComp24 = true;  //????????????????????????????????????
    private static final Distance kBackLeftXPosComp24 = Inches.of(-10.125);
    private static final Distance kBackLeftYPosComp24 = Inches.of(10.125);

    // Back Right
    private static final int kBackRightDriveMotorIdComp24 = 1;
    private static final int kBackRightSteerMotorIdComp24 = 2;
    private static final int kBackRightEncoderIdComp24 = 43;
    private static final Angle kBackRightEncoderOffsetComp24 = Rotations.of(-0.44140625);
    private static final boolean kBackRightSteerMotorInvertedComp24 = false;
    private static final boolean kBackRightEncoderInvertedComp24 = true; //??????????????????????????????????????

    private static final Distance kBackRightXPosComp24 = Inches.of(-10.125);
    private static final Distance kBackRightYPosComp24 = Inches.of(-10.125);


    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeftComp24 =
        ConstantCreatorComp24.createModuleConstants(
            kFrontLeftSteerMotorIdComp24, kFrontLeftDriveMotorIdComp24, kFrontLeftEncoderIdComp24, kFrontLeftEncoderOffsetComp24,
            kFrontLeftXPosComp24, kFrontLeftYPosComp24, kInvertLeftSide, kFrontLeftSteerMotorInvertedComp24, kFrontLeftEncoderInvertedComp24
        ).withDriveMotorInverted(false);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRightComp24 =
        ConstantCreatorComp24.createModuleConstants(
            kFrontRightSteerMotorIdComp24, kFrontRightDriveMotorIdComp24, kFrontRightEncoderIdComp24, kFrontRightEncoderOffsetComp24,
            kFrontRightXPosComp24, kFrontRightYPosComp24, kInvertRightSide, kFrontRightSteerMotorInvertedComp24, kFrontRightEncoderInvertedComp24
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeftComp24 =
        ConstantCreatorComp24.createModuleConstants(
            kBackLeftSteerMotorIdComp24, kBackLeftDriveMotorIdComp24, kBackLeftEncoderIdComp24, kBackLeftEncoderOffsetComp24,
            kBackLeftXPosComp24, kBackLeftYPosComp24, kInvertLeftSide, kBackLeftSteerMotorInvertedComp24, kBackLeftEncoderInvertedComp24
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRightComp24 =
        ConstantCreatorComp24.createModuleConstants(
            kBackRightSteerMotorIdComp24, kBackRightDriveMotorIdComp24, kBackRightEncoderIdComp24, kBackRightEncoderOffsetComp24,
            kBackRightXPosComp24, kBackRightYPosComp24, kInvertRightSide, kBackRightSteerMotorInvertedComp24, kBackRightEncoderInvertedComp24
        );

    /**************************************************************************************************
    * Define the Comp25 (2025 Competition Robot) Swerve Constants  
    ***************************************************************************************************/
    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatioComp25 = 2.6666666666666665;
    private static final double kDriveGearRatioComp25 = 5.142857142857143;
    private static final double kSteerGearRatioComp25 = 11.314285714285715;
  
    // CAN bus that the devices are located on; All swerve devices must share the same CAN bus
    public static final CANBus kCANBusComp25 = new CANBus("robot", "./logs/example.hoot");

    public static final SwerveDrivetrainConstants DrivetrainConstantsComp25 = new SwerveDrivetrainConstants()
       .withCANBusName(kCANBusComp25.getName())
       .withPigeon2Id(kPigeonId)
       .withPigeon2Configs(pigeonConfigs);

    // Simulated voltage necessary to overcome friction
    private static final Voltage kSteerFrictionVoltageComp25 = Volts.of(0.2);
    private static final Voltage kDriveFrictionVoltageComp25 = Volts.of(0.2);
    private static final Current kSlipCurrentComp25 =Amps.of(120.0); 

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGainsComp25 = new Slot0Configs()
    .withKP(100).withKI(0).withKD(0.5)
    .withKS(0.1).withKV(1.4).withKA(0)
    .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);
        ;
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGainsComp25 = new Slot0Configs()
    .withKP(0.1).withKI(0).withKD(0)
    .withKS(0).withKV(0.124);

    private static final ClosedLoopOutputType STEER_CLOSED_LOOP_OUTPUT_TYPEComp25 = ClosedLoopOutputType.Voltage;
    private static final ClosedLoopOutputType DRIVE_CLOSED_LOOP_OUTPUT_TYPEComp25 = ClosedLoopOutputType.Voltage;
    
    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorComp25 =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatioComp25)
            .withSteerMotorGearRatio(kSteerGearRatioComp25)
            .withCouplingGearRatio(kCoupleRatioComp25)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGainsComp25)
            .withDriveMotorGains(driveGainsComp25)
            .withSteerMotorClosedLoopOutput(STEER_CLOSED_LOOP_OUTPUT_TYPEComp25)
            .withDriveMotorClosedLoopOutput(DRIVE_CLOSED_LOOP_OUTPUT_TYPEComp25)
            .withSlipCurrent(kSlipCurrentComp25)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltageComp25)
            .withDriveFrictionVoltage(kDriveFrictionVoltageComp25);

    // Front Left
    private static final int kFrontLeftDriveMotorIdComp25 = 4;
    private static final int kFrontLeftSteerMotorIdComp25 = 3;
    private static final int kFrontLeftEncoderIdComp25 = 43;
    private static final Angle kFrontLeftEncoderOffsetComp25 = Rotations.of(-0.425537109375);
    private static final boolean kFrontLeftSteerMotorInvertedComp25 = true;
    private static final boolean kFrontLeftEncoderInvertedComp25 = false;  
    private static final Distance kFrontLeftXPosComp25 = Inches.of(11.5);
    private static final Distance kFrontLeftYPosComp25 = Inches.of(11.5);

    // Front Right
    private static final int kFrontRightDriveMotorIdComp25 = 8;
    private static final int kFrontRightSteerMotorIdComp25 = 7;
    private static final int kFrontRightEncoderIdComp25 = 41;
    private static final Angle kFrontRightEncoderOffsetComp25 =Rotations.of(0.103759765625);
    private static final boolean kFrontRightSteerMotorInvertedComp25 = true;
    private static final boolean kFrontRightEncoderInvertedComp25 = false; 
    private static final Distance kFrontRightXPosComp25 = Inches.of(11.5);
    private static final Distance kFrontRightYPosComp25 = Inches.of(-11.5);

    // Back Left
    private static final int kBackLeftDriveMotorIdComp25 = 2;
    private static final int kBackLeftSteerMotorIdComp25 = 1;
    private static final int kBackLeftEncoderIdComp25 = 42;
    private static final Angle kBackLeftEncoderOffsetComp25 = Rotations.of(-0.3125);
    private static final boolean kBackLeftSteerMotorInvertedComp25 = true;  
    private static final boolean kBackLeftEncoderInvertedComp25= false;  
    private static final Distance kBackLeftXPosComp25 = Inches.of(-11.5);
    private static final Distance kBackLeftYPosComp25 = Inches.of(11.5);

    // Back Right
    private static final int kBackRightDriveMotorIdComp25 = 6;
    private static final int kBackRightSteerMotorIdComp25 = 5;
    private static final int kBackRightEncoderIdComp25 = 40;
    private static final Angle kBackRightEncoderOffsetComp25 = Rotations.of(0.28125);
    private static final boolean kBackRightSteerMotorInvertedComp25 = true;
    private static final boolean kBackRightEncoderInvertedComp25 = false;

    private static final Distance kBackRightXPosComp25 = Inches.of(-11.5);
    private static final Distance kBackRightYPosComp25 = Inches.of(-11.5);


    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeftComp25 =
        ConstantCreatorComp25.createModuleConstants(
            kFrontLeftSteerMotorIdComp25, kFrontLeftDriveMotorIdComp25, kFrontLeftEncoderIdComp25, kFrontLeftEncoderOffsetComp25,
            kFrontLeftXPosComp25, kFrontLeftYPosComp25, kInvertLeftSide, kFrontLeftSteerMotorInvertedComp25, kFrontLeftEncoderInvertedComp25
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRightComp25 =
        ConstantCreatorComp25.createModuleConstants(
            kFrontRightSteerMotorIdComp25, kFrontRightDriveMotorIdComp25, kFrontRightEncoderIdComp25, kFrontRightEncoderOffsetComp25,
            kFrontRightXPosComp25, kFrontRightYPosComp25, kInvertRightSide, kFrontRightSteerMotorInvertedComp25, kFrontRightEncoderInvertedComp25
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeftComp25 =
        ConstantCreatorComp25.createModuleConstants(
            kBackLeftSteerMotorIdComp25, kBackLeftDriveMotorIdComp25, kBackLeftEncoderIdComp25, kBackLeftEncoderOffsetComp25,
            kBackLeftXPosComp25, kBackLeftYPosComp25, kInvertLeftSide, kBackLeftSteerMotorInvertedComp25, kBackLeftEncoderInvertedComp25
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRightComp25 =
        ConstantCreatorComp25.createModuleConstants(
            kBackRightSteerMotorIdComp25, kBackRightDriveMotorIdComp25, kBackRightEncoderIdComp25, kBackRightEncoderOffsetComp25,
            kBackRightXPosComp25, kBackRightYPosComp25, kInvertRightSide, kBackRightSteerMotorInvertedComp25, kBackRightEncoderInvertedComp25
        );


    /**************************************************************************************************
    * Define the Devbot (Development Robot) Swerve Constants  
    ***************************************************************************************************/
    
    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatioDevbot = 3.5714285714285716;
    private static final double kDriveGearRatioDevbot = 5.01;
    private static final double kSteerGearRatioDevbot = 13.3714;

    // CAN bus that the devices are located on; All swerve devices must share the same CAN bus
    public static final CANBus kCANBusDevbot = new CANBus("robot", "./logs/example.hoot");

    public static final SwerveDrivetrainConstants DrivetrainConstantsDevbot = new SwerveDrivetrainConstants()
        .withCANBusName(kCANBusDevbot.getName())
        .withPigeon2Id(kPigeonId)
        .withPigeon2Configs(pigeonConfigs);

    // The steer motor uses any SwerveModule.SteerRequestType control request with the
    // output type specified by SwerveModuleConstants.SteerMotorClosedLoopOutput
    private static final Slot0Configs steerGainsDevbot = new Slot0Configs()
        .withKP(2250).withKI(0).withKD(20)
        .withKS(2).withKV(0).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
        ;
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGainsDevbot = new Slot0Configs()
        .withKP(5.3).withKI(0).withKD(0)
        .withKS(5.5).withKV(0.3);

        // private static final Slot0Configs driveGains = new Slot0Configs()
        // .withKP(0.1).withKI(0).withKD(0)
        // .withKS(0).withKV(0.124);

    //private static final Slot0Configs driveGainsVoltageDevBot = new Slot0Configs()
    //    .withKP(0.1).withKI(0).withKD(0)
    //    .withKS(0).withKV(0.124);



    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorDevbot =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatioDevbot)
            .withSteerMotorGearRatio(kSteerGearRatioDevbot)
            .withCouplingGearRatio(kCoupleRatioDevbot)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGainsDevbot)
            .withDriveMotorGains(driveGainsDevbot)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Front Left
    private static final int kFrontLeftDriveMotorIdDevbot = 8;
    private static final int kFrontLeftSteerMotorIdDevbot = 7;
    private static final int kFrontLeftEncoderIdDevbot = 41;
    private static final Angle kFrontLeftEncoderOffsetDevbot = Rotations.of(-0.12255859375);
    private static final boolean kFrontLeftSteerMotorInvertedDevbot = true;
    private static final boolean kFrontLeftEncoderInvertedDevbot = false;

    private static final Distance kFrontLeftXPosDevbot = Inches.of(11);
    private static final Distance kFrontLeftYPosDevbot = Inches.of(11);

    // Front Right
    private static final int kFrontRightDriveMotorIdDevbot = 6;
    private static final int kFrontRightSteerMotorIdDevbot = 5;
    private static final int kFrontRightEncoderIdDevbot = 40;
    private static final Angle kFrontRightEncoderOffsetDevbot = Rotations.of(-0.442138671875);
    private static final boolean kFrontRightSteerMotorInvertedDevbot = true;
    private static final boolean kFrontRightEncoderInvertedDevbot = false;

    private static final Distance kFrontRightXPosDevbot = Inches.of(11);
    private static final Distance kFrontRightYPosDevbot = Inches.of(-11);

    // Back Left
    private static final int kBackLeftDriveMotorIdDevbot = 4;
    private static final int kBackLeftSteerMotorIdDevbot = 3;
    private static final int kBackLeftEncoderIdDevbot = 43;
    private static final Angle kBackLeftEncoderOffsetDevbot = Rotations.of(-0.17626953125);
    private static final boolean kBackLeftSteerMotorInvertedDevbot = true;
    private static final boolean kBackLeftEncoderInvertedDevbot = false;

    private static final Distance kBackLeftXPosDevbot = Inches.of(-11);
    private static final Distance kBackLeftYPosDevbot = Inches.of(11);

    // Back Right
    private static final int kBackRightDriveMotorIdDevbot = 2;
    private static final int kBackRightSteerMotorIdDevbot = 1;
    private static final int kBackRightEncoderIdDevbot = 42;
    private static final Angle kBackRightEncoderOffsetDevbot = Rotations.of(0.443359375);
    private static final boolean kBackRightSteerMotorInvertedDevbot = true;
    private static final boolean kBackRightEncoderInvertedDevbot = false;

    private static final Distance kBackRightXPosDevbot = Inches.of(-11);
    private static final Distance kBackRightYPosDevbot = Inches.of(-11);
    


    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeftDevbot =
        ConstantCreatorDevbot.createModuleConstants(
            kFrontLeftSteerMotorIdDevbot, kFrontLeftDriveMotorIdDevbot, kFrontLeftEncoderIdDevbot, kFrontLeftEncoderOffsetDevbot,
            kFrontLeftXPosDevbot, kFrontLeftYPosDevbot, kInvertLeftSide, kFrontLeftSteerMotorInvertedDevbot, kFrontLeftEncoderInvertedDevbot
        ).withDriveMotorInverted(false);
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRightDevbot =
        ConstantCreatorDevbot.createModuleConstants(
            kFrontRightSteerMotorIdDevbot, kFrontRightDriveMotorIdDevbot, kFrontRightEncoderIdDevbot, kFrontRightEncoderOffsetDevbot,
            kFrontRightXPosDevbot, kFrontRightYPosDevbot, kInvertRightSide, kFrontRightSteerMotorInvertedDevbot, kFrontRightEncoderInvertedDevbot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeftDevbot =
        ConstantCreatorDevbot.createModuleConstants(
            kBackLeftSteerMotorIdDevbot, kBackLeftDriveMotorIdDevbot, kBackLeftEncoderIdDevbot, kBackLeftEncoderOffsetDevbot,
            kBackLeftXPosDevbot, kBackLeftYPosDevbot, kInvertLeftSide, kBackLeftSteerMotorInvertedDevbot, kBackLeftEncoderInvertedDevbot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRightDevbot =
        ConstantCreatorDevbot.createModuleConstants(
            kBackRightSteerMotorIdDevbot, kBackRightDriveMotorIdDevbot, kBackRightEncoderIdDevbot, kBackRightEncoderOffsetDevbot,
            kBackRightXPosDevbot, kBackRightYPosDevbot, kInvertRightSide, kBackRightSteerMotorInvertedDevbot, kBackRightEncoderInvertedDevbot
        );



    /**************************************************************************************************
    * Define the Cambot (Camera Robot) Swerve Constants  
    ***************************************************************************************************/
 
    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatioCambot = 3.5714285714285716;
    private static final double kDriveGearRatioCambot = 6.746031746031747;
    private static final double kSteerGearRatioCambot = 21.428571428571427;

    private static final boolean kInvertLeftSideCambot = false;
    private static final boolean kInvertRightSideCambot = true;

    // CAN bus that the devices are located on; All swerve devices must share the same CAN bus
    public static final CANBus kCANBusCambot = new CANBus("robot", "./logs/example.hoot");

    public static final SwerveDrivetrainConstants DrivetrainConstantsCambot = new SwerveDrivetrainConstants()
     .withCANBusName(kCANBusCambot.getName())
     .withPigeon2Id(kPigeonId)
     .withPigeon2Configs(pigeonConfigs);

    // DEFAULT GAINS PULLED FROM CTRE EXAMPLE CODE
    private static final Slot0Configs steerGainsCameraBot = new Slot0Configs()
        .withKP(100).withKI(0).withKD(0.5)
        .withKS(0.1).withKV(1.91).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign);

    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    //private static final Slot0Configs driveGainsCameraBot = new Slot0Configs()
    //    .withKP(0.1).withKI(0).withKD(0)
    //    .withKS(0).withKV(0.124);

    private static final Slot0Configs driveGainsVoltageCameraBot = new Slot0Configs()
        .withKP(0.1).withKI(0).withKD(0)
        .withKS(0).withKV(0.124);


    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorCambot =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatioCambot)
            .withSteerMotorGearRatio(kSteerGearRatioCambot)
            .withCouplingGearRatio(kCoupleRatioCambot)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGainsCameraBot)
            .withDriveMotorGains(driveGainsVoltageCameraBot)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutputCambot)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Front Left
    private static final int kFrontLeftDriveMotorIdCambot = 6;
    private static final int kFrontLeftSteerMotorIdCambot = 8;
    private static final int kFrontLeftEncoderIdCambot = 18;
    private static final Angle kFrontLeftEncoderOffsetCambot = Rotations.of(0.41552734375);
    private static final boolean kFrontLeftSteerMotorInvertedCambot = false;
    private static final boolean kFrontLeftEncoderInvertedCambot = false;

    private static final Distance kFrontLeftXPosCambot = Inches.of(8.75);
    private static final Distance kFrontLeftYPosCambot = Inches.of(8.75);

    // Front Right
    private static final int kFrontRightDriveMotorIdCambot = 2;
    private static final int kFrontRightSteerMotorIdCambot = 4;
    private static final int kFrontRightEncoderIdCambot = 14;
    private static final Angle kFrontRightEncoderOffsetCambot = Rotations.of(-0.361572265625);
    private static final boolean kFrontRightSteerMotorInvertedCambot = false;
    private static final boolean kFrontRightEncoderInvertedCambot = false;

    private static final Distance kFrontRightXPosCambot = Inches.of(8.75);
    private static final Distance kFrontRightYPosCambot = Inches.of(-8.75);

    // Back Left
    private static final int kBackLeftDriveMotorIdCambot = 5;
    private static final int kBackLeftSteerMotorIdCambot = 7;
    private static final int kBackLeftEncoderIdCambot = 17;
    private static final Angle kBackLeftEncoderOffsetCambot = Rotations.of(-0.162841796875);
    private static final boolean kBackLeftSteerMotorInvertedCambot = false;
    private static final boolean kBackLeftEncoderInvertedCambot = false;

    private static final Distance kBackLeftXPosCambot = Inches.of(-8.75);
    private static final Distance kBackLeftYPosCambot = Inches.of(8.75);

    // Back Right
    private static final int kBackRightDriveMotorIdCambot = 1;
    private static final int kBackRightSteerMotorIdCambot = 3;
    private static final int kBackRightEncoderIdCambot = 13;
    private static final Angle kBackRightEncoderOffsetCambot = Rotations.of(0.43896484375);
    private static final boolean kBackRightSteerMotorInvertedCambot = false;
    private static final boolean kBackRightEncoderInvertedCambot = false; 

    private static final Distance kBackRightXPosCambot = Inches.of(-8.75);
    private static final Distance kBackRightYPosCambot = Inches.of(-8.75);
    
    // Load the Swerve Module Objects 
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeftCambot =
        ConstantCreatorCambot.createModuleConstants(
            kFrontLeftSteerMotorIdCambot, kFrontLeftDriveMotorIdCambot, kFrontLeftEncoderIdCambot, kFrontLeftEncoderOffsetCambot,
            kFrontLeftXPosCambot, kFrontLeftYPosCambot, kInvertLeftSideCambot, kFrontLeftSteerMotorInvertedCambot, kFrontLeftEncoderInvertedCambot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRightCambot =
        ConstantCreatorCambot.createModuleConstants(
            kFrontRightSteerMotorIdCambot, kFrontRightDriveMotorIdCambot, kFrontRightEncoderIdCambot, kFrontRightEncoderOffsetCambot,
            kFrontRightXPosCambot, kFrontRightYPosCambot, kInvertRightSideCambot, kFrontRightSteerMotorInvertedCambot, kFrontRightEncoderInvertedCambot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeftCambot =
        ConstantCreatorCambot.createModuleConstants(
            kBackLeftSteerMotorIdCambot, kBackLeftDriveMotorIdCambot, kBackLeftEncoderIdCambot, kBackLeftEncoderOffsetCambot,
            kBackLeftXPosCambot, kBackLeftYPosCambot, kInvertLeftSideCambot, kBackLeftSteerMotorInvertedCambot, kBackLeftEncoderInvertedCambot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRightCambot =
        ConstantCreatorCambot.createModuleConstants(
            kBackRightSteerMotorIdCambot, kBackRightDriveMotorIdCambot, kBackRightEncoderIdCambot, kBackRightEncoderOffsetCambot,
            kBackRightXPosCambot, kBackRightYPosCambot, kInvertRightSideCambot, kBackRightSteerMotorInvertedCambot, kBackRightEncoderInvertedCambot
        );


    /**************************************************************************************************
    * Define the CompBot Swerve Constants  
    ***************************************************************************************************/
        
    // Every 1 rotation of the azimuth results in kCoupleRatio drive motor turns;
    // This may need to be tuned to your individual robot
    private static final double kCoupleRatioCompbot = 4.909090909090909;
    private static final double kDriveGearRatioCompbot = 6.976076555023923;
    private static final double kSteerGearRatioCompbot = 12.1;

     // CAN bus that the devices are located on; All swerve devices must share the same CAN bus
     public static final CANBus kCANBusCompbot = new CANBus("robot", "./logs/example.hoot");

     public static final SwerveDrivetrainConstants DrivetrainConstantsCompbot = new SwerveDrivetrainConstants()
         .withCANBusName(kCANBusCompbot.getName())
         .withPigeon2Id(kPigeonId)
         .withPigeon2Configs(pigeonConfigs);

    private static final Slot0Configs steerGainsCompbot = new Slot0Configs()
        .withKP(2250).withKI(0).withKD(20)
        .withKS(2).withKV(0).withKA(0)
        .withStaticFeedforwardSign(StaticFeedforwardSignValue.UseClosedLoopSign)
        ;
    // When using closed-loop control, the drive motor uses the control
    // output type specified by SwerveModuleConstants.DriveMotorClosedLoopOutput
    private static final Slot0Configs driveGainsCompbot = new Slot0Configs()
        .withKP(5.3).withKI(0).withKD(0)
        .withKS(5.5).withKV(0.3);

    // private static final Slot0Configs driveGains = new Slot0Configs()
    // .withKP(0.1).withKI(0).withKD(0)
    // .withKS(0).withKV(0.124);

    //private static final Slot0Configs driveGainsVoltageCompbot = new Slot0Configs()
    //    .withKP(0.1).withKI(0).withKD(0)
    //    .withKS(0).withKV(0.124);

    private static final SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> ConstantCreatorCompbot =
        new SwerveModuleConstantsFactory<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration>()
            .withDriveMotorGearRatio(kDriveGearRatioCompbot)
            .withSteerMotorGearRatio(kSteerGearRatioCompbot)
            .withCouplingGearRatio(kCoupleRatioCompbot)
            .withWheelRadius(kWheelRadius)
            .withSteerMotorGains(steerGainsCompbot)
            .withDriveMotorGains(driveGainsCompbot)
            .withSteerMotorClosedLoopOutput(kSteerClosedLoopOutput)
            .withDriveMotorClosedLoopOutput(kDriveClosedLoopOutput)
            .withSlipCurrent(kSlipCurrent)
            .withSpeedAt12Volts(kSpeedAt12Volts)
            .withDriveMotorType(kDriveMotorType)
            .withSteerMotorType(kSteerMotorType)
            .withFeedbackSource(kSteerFeedbackType)
            .withDriveMotorInitialConfigs(driveInitialConfigs)
            .withSteerMotorInitialConfigs(steerInitialConfigs)
            .withEncoderInitialConfigs(encoderInitialConfigs)
            .withSteerInertia(kSteerInertia)
            .withDriveInertia(kDriveInertia)
            .withSteerFrictionVoltage(kSteerFrictionVoltage)
            .withDriveFrictionVoltage(kDriveFrictionVoltage);

    // Front Left
    private static final int kFrontLeftDriveMotorIdCompbot = 8;
    private static final int kFrontLeftSteerMotorIdCompbot = 7;
    private static final int kFrontLeftEncoderIdCompbot = 41;
    private static final Angle kFrontLeftEncoderOffsetCompbot = Rotations.of(-0.267822);
    private static final boolean kFrontLeftSteerMotorInvertedCompbot = false;
    private static final boolean kFrontLeftEncoderInvertedCompbot = false;

    private static final Distance kFrontLeftXPosCompbot = Inches.of(11.5);
    private static final Distance kFrontLeftYPosCompbot = Inches.of(11.5);

    // Front Right
    private static final int kFrontRightDriveMotorIdCompbot = 6;
    private static final int kFrontRightSteerMotorIdCompbot = 5;
    private static final int kFrontRightEncoderIdCompbot = 40;
    private static final Angle kFrontRightEncoderOffsetCompbot = Rotations.of(0.3642578125);
    private static final boolean kFrontRightSteerMotorInvertedCompbot = false;
    private static final boolean kFrontRightEncoderInvertedCompbot = false;

    private static final Distance kFrontRightXPosCompbot = Inches.of(11.5);
    private static final Distance kFrontRightYPosCompbot = Inches.of(-11.5);

    // Back Left
    private static final int kBackLeftDriveMotorIdCompbot = 4;
    private static final int kBackLeftSteerMotorIdCompbot = 3;
    private static final int kBackLeftEncoderIdCompbot = 43;
    private static final Angle kBackLeftEncoderOffsetCompbot = Rotations.of(-0.1357421875);
    private static final boolean kBackLeftSteerMotorInvertedCompbot = false;
    private static final boolean kBackLeftEncoderInvertedCompbot = false;

    private static final Distance kBackLeftXPosCompbot = Inches.of(-11.5);
    private static final Distance kBackLeftYPosCompbot = Inches.of(11.5);

    // Back Right
    private static final int kBackRightDriveMotorIdCompbot = 2;
    private static final int kBackRightSteerMotorIdCompbot = 1;
    private static final int kBackRightEncoderIdCompbot = 42;
    private static final Angle kBackRightEncoderOffsetCompbot = Rotations.of(-0.0908203125);
    private static final boolean kBackRightSteerMotorInvertedCompbot = false;
    private static final boolean kBackRightEncoderInvertedCompbot = false;

    private static final Distance kBackRightXPosCompbot = Inches.of(-11.5);
    private static final Distance kBackRightYPosCompbot = Inches.of(-11.5);

    // Load the Swerve Module Objects
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontLeftCompbot =
        ConstantCreatorCompbot.createModuleConstants(
            kFrontLeftSteerMotorIdCompbot, kFrontLeftDriveMotorIdCompbot, kFrontLeftEncoderIdCompbot, kFrontLeftEncoderOffsetCompbot,
            kFrontLeftXPosCompbot, kFrontLeftYPosCompbot, kInvertLeftSide, kFrontLeftSteerMotorInvertedCompbot, kFrontLeftEncoderInvertedCompbot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FrontRightCompbot =
        ConstantCreatorCompbot.createModuleConstants(
            kFrontRightSteerMotorIdCompbot, kFrontRightDriveMotorIdCompbot, kFrontRightEncoderIdCompbot, kFrontRightEncoderOffsetCompbot,
            kFrontRightXPosCompbot, kFrontRightYPosCompbot, kInvertRightSide, kFrontRightSteerMotorInvertedCompbot, kFrontRightEncoderInvertedCompbot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackLeftCompbot =
        ConstantCreatorCompbot.createModuleConstants(
            kBackLeftSteerMotorIdCompbot, kBackLeftDriveMotorIdCompbot, kBackLeftEncoderIdCompbot, kBackLeftEncoderOffsetCompbot,
            kBackLeftXPosCompbot, kBackLeftYPosCompbot, kInvertLeftSide, kBackLeftSteerMotorInvertedCompbot, kBackLeftEncoderInvertedCompbot
        );
    public static final SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BackRightCompbot =
        ConstantCreatorCompbot.createModuleConstants(
            kBackRightSteerMotorIdCompbot, kBackRightDriveMotorIdCompbot, kBackRightEncoderIdCompbot, kBackRightEncoderOffsetCompbot,
            kBackRightXPosCompbot, kBackRightYPosCompbot, kInvertRightSide, kBackRightSteerMotorInvertedCompbot, kBackRightEncoderInvertedCompbot
        );



/**************************************************************************************************
 * Set Final Drive Train Objects based on Robot Selected in Constants 
***************************************************************************************************/

    public static final DriveConfig DriveConfig =
        switch (Constants.getRobot()) {
            case COMPBOT -> new DriveConfig(
                FrontLeftCompbot, 
                FrontRightCompbot, 
                BackLeftCompbot, 
                BackRightCompbot,
                DrivetrainConstantsCompbot,
                kSpeedAt12Volts.in(MetersPerSecond),
                kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits,
                steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation,
                autoPidConstantsTheta,
                teleopPidConstantsTheta
                );

            case DEVBOT -> new DriveConfig(
                FrontLeftDevbot, 
                FrontRightDevbot, 
                BackLeftDevbot, 
                BackRightDevbot,
                DrivetrainConstantsDevbot,
                kSpeedAt12Volts.in(MetersPerSecond),
                kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits,
                steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation,
                autoPidConstantsTheta,
                teleopPidConstantsTheta
                );

            case CAMERABOT -> new DriveConfig(
                FrontLeftCambot, 
                FrontRightCambot, 
                BackLeftCambot, 
                BackRightCambot,
                DrivetrainConstantsCambot,
                kSpeedAt12Volts.in(MetersPerSecond),
                kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits,
                steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation,
                autoPidConstantsTheta,
                teleopPidConstantsTheta
                );

                
            case COMP24 -> new DriveConfig(
                FrontLeftComp24, 
                FrontRightComp24, 
                BackLeftComp24, 
                BackRightComp24,
                DrivetrainConstantsComp24,
                kSpeedAt12Volts.in(MetersPerSecond),
                kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits,
                steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation,
                autoPidConstantsTheta,
                teleopPidConstantsTheta
                );

            case COMP25 -> new DriveConfig(
                FrontLeftComp25, 
                FrontRightComp25, 
                BackLeftComp25, 
                BackRightComp25,
                DrivetrainConstantsComp25,
                kSpeedAt12Volts.in(MetersPerSecond),
                kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits,
                steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation,
                autoPidConstantsTheta,
                teleopPidConstantsTheta
                );

            case MINIBOT -> new DriveConfig(
                FrontLeftMinibot, 
                FrontRightMinibot, 
                BackLeftMinibot, 
                BackRightMinibot,
                DrivetrainConstantsMinibot,
                kSpeedAt12Volts.in(MetersPerSecond),
                kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits,
                steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation,
                autoPidConstantsTheta,
                teleopPidConstantsTheta
                );


            case SIMBOT -> new DriveConfig(
                FrontLeftCambot, 
                FrontRightCambot, 
                BackLeftCambot, 
                BackRightCambot,
                DrivetrainConstantsCambot,
                kSpeedAt12Volts.in(MetersPerSecond),
                kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits,
                steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation,
                autoPidConstantsTheta,
                teleopPidConstantsTheta
                );
        };


    /*************************************************************************************************
    * Define the Drive Config Objects used in the Drive Program 
    ***************************************************************************************************/
    public record DriveConfig(
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_LEFT, 
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> FRONT_RIGHT, 
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_LEFT, 
        SwerveModuleConstants<TalonFXConfiguration, TalonFXConfiguration, CANcoderConfiguration> BACK_RIGHT,
        SwerveDrivetrainConstants DRIVETRAIN,
        double MAX_VELOCITY,
        double MAX_ANGULAR_VELOCITY,
        CurrentLimitsConfigs DRIVE_CURRENT,
        CurrentLimitsConfigs AZIMUTH_CURRENT,
        PIDConstants AUTON_TRANSLATION_PID,
        PIDConstants AUTON_ROTATION_PID,
        PIDConstants TELEOP_ROTATION_PID
        ) {}

        //Comp Gains:
        // public static final double bargePosRedFar = 9.925;
        // public static final double bargePosRedClose = 9.75;
        // public static final double bargePosBlueFar = 7.86;
        // public static final double bargePosBlueClose = 7.685;

        //GM Gains:
        public static final double bargePosRedFar = 10.0;
        public static final double bargePosRedClose = 9.75;
        public static final double bargePosBlueFar = 7.86;
        public static final double bargePosBlueClose = 7.61;


}

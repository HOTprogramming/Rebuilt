package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.*;
import com.ctre.phoenix6.swerve.*;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

import frc.robot.Constants;
import frc.robot.subsystems.Drivetrain.constants.*;

public class DriveConstants {

    // ==================== Shared / Global Constants ====================
    public static final double slowModeMultiplier = 0.5;
    public static final double kTranslationRateLimiter = 3.5;
    public static final double kRotationRateLimiter = 5.0;
    public static final double OTF_end_tolerance = 0.01;

    public static final double auto_align_tolerance = 0.01;

    private static final PIDConstants autoPidConstantsTranslation = new PIDConstants(5, 0, 0);
    private static final PIDConstants autoPidConstantsTheta = new PIDConstants(7, 0, 0);
    private static final PIDConstants teleopPidConstantsTheta = new PIDConstants(10, 0, 0.5);

    public static final TalonFXConfiguration driveInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(70)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(30)
                .withSupplyCurrentLowerTime(1)
        ).withTorqueCurrent(
            new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(70)
                .withPeakReverseTorqueCurrent(-70)
        );

    public static final TalonFXConfiguration steerInitialConfigs = new TalonFXConfiguration()
        .withCurrentLimits(
            new CurrentLimitsConfigs()
                .withStatorCurrentLimit(60)
                .withStatorCurrentLimitEnable(true)
                .withSupplyCurrentLimit(40)
                .withSupplyCurrentLimitEnable(true)
                .withSupplyCurrentLowerLimit(40)
                .withSupplyCurrentLowerTime(1)
        ).withTorqueCurrent(
            new TorqueCurrentConfigs()
                .withPeakForwardTorqueCurrent(60)
                .withPeakReverseTorqueCurrent(-60)
        );

    public static final CANcoderConfiguration encoderInitialConfigs = new CANcoderConfiguration();

    // ==================== Robot Selection Switch ====================

    public static final DriveConfig DriveConfig =
        switch (Constants.getRobot()) {
            case COMPBOT -> new DriveConfig(
                CompBotConstants.FrontLeft, CompBotConstants.FrontRight,
                CompBotConstants.BackLeft, CompBotConstants.BackRight,
                CompBotConstants.DrivetrainConstants,
                CompBotConstants.kSpeedAt12Volts.in(MetersPerSecond), CompBotConstants.kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits, steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation, autoPidConstantsTheta, teleopPidConstantsTheta);

            case CAMERABOT -> new DriveConfig(
                CameraBotConstants.FrontLeft, CameraBotConstants.FrontRight,
                CameraBotConstants.BackLeft, CameraBotConstants.BackRight,
                CameraBotConstants.DrivetrainConstants,
                CameraBotConstants.kSpeedAt12Volts.in(MetersPerSecond), CameraBotConstants.kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits, steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation, autoPidConstantsTheta, teleopPidConstantsTheta);

            case COMP24 -> new DriveConfig(
                Comp24Constants.FrontLeft, Comp24Constants.FrontRight,
                Comp24Constants.BackLeft, Comp24Constants.BackRight,
                Comp24Constants.DrivetrainConstants,
                Comp24Constants.kSpeedAt12Volts.in(MetersPerSecond), Comp24Constants.kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits, steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation, autoPidConstantsTheta, teleopPidConstantsTheta);

            case COMP25 -> new DriveConfig(
                Comp25Constants.FrontLeft, Comp25Constants.FrontRight,
                Comp25Constants.BackLeft, Comp25Constants.BackRight,
                Comp25Constants.DrivetrainConstants,
                Comp25Constants.kSpeedAt12Volts.in(MetersPerSecond), Comp25Constants.kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits, steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation, autoPidConstantsTheta, teleopPidConstantsTheta);

            case ALPHABOT -> new DriveConfig(
                AlphaBotConstants.FrontLeft, AlphaBotConstants.FrontRight,
                AlphaBotConstants.BackLeft, AlphaBotConstants.BackRight,
                AlphaBotConstants.DrivetrainConstants,
                AlphaBotConstants.kSpeedAt12Volts.in(MetersPerSecond), AlphaBotConstants.kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits, steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation, autoPidConstantsTheta, teleopPidConstantsTheta);

            case MINIBOT -> new DriveConfig(
                MiniBotConstants.FrontLeft, MiniBotConstants.FrontRight,
                MiniBotConstants.BackLeft, MiniBotConstants.BackRight,
                MiniBotConstants.DrivetrainConstants,
                MiniBotConstants.kSpeedAt12Volts.in(MetersPerSecond), MiniBotConstants.kMaxAngularVelocity.in(RadiansPerSecond),
                driveInitialConfigs.CurrentLimits, steerInitialConfigs.CurrentLimits,
                autoPidConstantsTranslation, autoPidConstantsTheta, teleopPidConstantsTheta);
        };

    // Down here so it can get the correct MAX_VELOCITY from the DriveConfig record above
    public static final TrapezoidProfile.Constraints DEFAULT_XY_CONSTRAINTS =
        new TrapezoidProfile.Constraints(DriveConfig.MAX_VELOCITY(), 5);

    // ==================== DriveConfig Record ====================
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
}

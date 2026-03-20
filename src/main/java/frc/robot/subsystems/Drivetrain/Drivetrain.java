package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;
import java.util.function.Supplier;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotState;
import frc.robot.subsystems.Drivetrain.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain.constants.DrivetrainConstants.TunerSwerveDrivetrain;

public class Drivetrain extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.004;
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    public boolean tipped = false;

	private final SlewRateLimiter xLimiter = new SlewRateLimiter(3.5);
	private final SlewRateLimiter yLimiter = new SlewRateLimiter(3.5);
	private final SlewRateLimiter rotLimiter = new SlewRateLimiter(5);

	private double MaxSpeed = 1.0 * DrivetrainConstants.kSpeedAt12Volts.in(MetersPerSecond);
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond);

    private final SwerveRequest.FieldCentric teleopDrive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage);

	private final SwerveRequest.FieldCentricFacingAngle FIELD_CENTRIC_FACING_ANGLE = new SwerveRequest.FieldCentricFacingAngle()
			.withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1)
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
			.withHeadingPID(7, 0, 0);

    private final DrivetrainTelemetry logger = new DrivetrainTelemetry(MaxSpeed);

    public Drivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();

		registerTelemetry(logger::telemeterize);
    }

    private void configureAutoBuilder() {
        try {
            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                () -> getState().Pose,
                this::resetPose,
                () -> getState().Speeds,
                (speeds, feedforwards) -> setControl(
                    new SwerveRequest.ApplyRobotSpeeds().withSpeeds(ChassisSpeeds.discretize(speeds, 0.020))
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    new PIDConstants(10, 0, 0),
                    new PIDConstants(7, 0, 0)
                ),
                config,
                () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

	public void teleopDrive(double driveX, double driveY, double driveTheta) {
		double rampedX = xLimiter.calculate((driveX * Math.abs(driveX)));
		double rampedY = yLimiter.calculate((driveY * Math.abs(driveY)));
		double rampedTheta = rotLimiter.calculate(driveTheta * Math.abs(driveTheta));

		setControl(teleopDrive
			.withVelocityX(rampedX * MaxSpeed)
			.withVelocityY(rampedY * MaxSpeed)
			.withRotationalRate(rampedTheta * MaxAngularRate));
	}

	public void aim(double driveX, double driveY) {
		setControl(FIELD_CENTRIC_FACING_ANGLE
			.withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * MaxSpeed * .25)
			.withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * MaxSpeed * .25)
			.withTargetDirection(Rotation2d.fromRadians(RobotState.turretAngle + Math.PI)));
	}

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? Rotation2d.k180deg
                        : Rotation2d.kZero
                );
            });
        }

        tipped = Math.abs(getPigeon2().getPitch().getValueAsDouble()) > 2 || Math.abs(getPigeon2().getPitch().getValueAsDouble()) > 2;

		RobotState.pose = getState().Pose;
		RobotState.speeds = getState().Speeds;
	}

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }
}
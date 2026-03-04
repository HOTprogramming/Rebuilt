package frc.robot.subsystems.Drivetrain;

import static frc.robot.subsystems.Drivetrain.DriveConstants.*;
import frc.robot.Constants;
import frc.robot.RobotState;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

class DriveIOdata {
	public SwerveDriveState state = new SwerveDriveState();

	public Rotation3d pigeon;

	public double frontLeftDrivePositionRads = 0.0;
	public double frontLeftDriveVelocityRpm = 0.0;
	public double frontLeftDriveAppliedVolts = 0.0;
	public double frontLeftDriveSupplyCurrentAmps = 0.0;
	public double frontLeftDriveTorqueCurrentAmps = 0.0;
	public double frontLeftDriveTempCelsius = 0.0;

	public double frontRightDrivePositionRads = 0.0;
	public double frontRightDriveVelocityRpm = 0.0;
	public double frontRightDriveAppliedVolts = 0.0;
	public double frontRightDriveSupplyCurrentAmps = 0.0;
	public double frontRightDriveTorqueCurrentAmps = 0.0;
	public double frontRightDriveTempCelsius = 0.0;

	public double backLeftDrivePositionRads = 0.0;
	public double backLeftDriveVelocityRpm = 0.0;
	public double backLeftDriveAppliedVolts = 0.0;
	public double backLeftDriveSupplyCurrentAmps = 0.0;
	public double backLeftDriveTorqueCurrentAmps = 0.0;
	public double backLeftDriveTempCelsius = 0.0;

	public double backRightDrivePositionRads = 0.0;
	public double backRightDriveVelocityRpm = 0.0;
	public double backRightDriveAppliedVolts = 0.0;
	public double backRightDriveSupplyCurrentAmps = 0.0;
	public double backRightDriveTorqueCurrentAmps = 0.0;
	public double backRightDriveTempCelsius = 0.0;

	public Translation2d[] m_moduleLocations;
}

public class Drive extends SubsystemBase {
	private double thetaP = 10.0;
	private double thetaI = 0.0;
	private double thetaD = 0.2;

	private final PIDController thetaController = new PIDController(thetaP, thetaI, thetaD);

	// NetworkTables
	private NetworkTable tuningTable;

	private DoublePublisher thetaP_pub;
	private DoublePublisher thetaI_pub;
	private DoublePublisher thetaD_pub;

	private edu.wpi.first.networktables.DoubleSubscriber thetaP_sub;
	private edu.wpi.first.networktables.DoubleSubscriber thetaI_sub;
	private edu.wpi.first.networktables.DoubleSubscriber thetaD_sub;

	private final SlewRateLimiter xLimiter = new SlewRateLimiter(kTranslationRateLimiter);
	private final SlewRateLimiter yLimiter = new SlewRateLimiter(kTranslationRateLimiter);
	private final SlewRateLimiter rotLimiter = new SlewRateLimiter(kRotationRateLimiter);

	private DriveKraken driveKraken;
	private DriveIOdata iOdata;

	private PathConstraints constraints;

	private double rampedX;
	private double rampedY;
	private double rampedTheta;

	private final SwerveRequest.FieldCentric FIELD_CENTRIC = new SwerveRequest.FieldCentric()
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage);
	private final SwerveRequest.FieldCentricFacingAngle FIELD_CENTRIC_FACING_ANGLE = new SwerveRequest.FieldCentricFacingAngle()
		.withDriveRequestType(DriveRequestType.OpenLoopVoltage)
		.withHeadingPID(10, 0, 0);
	private final SwerveRequest.RobotCentric ROBOT_CENTRIC = new SwerveRequest.RobotCentric();
	private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds()
		.withDriveRequestType(DriveRequestType.Velocity);

	private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
	private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

	private DoublePublisher robotReqSpeed;
	private DoublePublisher robotRealSpeed;
	private DoubleArrayPublisher robotPose;

	public Drive(DriveKraken driveKraken) {
		this.driveKraken = driveKraken;

		this.iOdata = driveKraken.update();

		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		NetworkTable table = inst.getTable("HOT/Drive");

		robotReqSpeed = table.getDoubleTopic("Robot Requested Speed").publish();
		robotRealSpeed = table.getDoubleTopic("Robot Real Speed").publish();
		robotPose = table.getDoubleArrayTopic("Robot Pose").publish();

		thetaController.enableContinuousInput(-Math.PI, Math.PI);

		configurePathPlanner();

		constraints = new PathConstraints(5.0, 3.0, 2 * Math.PI, 4 * Math.PI);
	}

	private void UpdateTelemetry() {
		robotReqSpeed.set(Math.hypot(FIELD_CENTRIC.VelocityX, FIELD_CENTRIC.VelocityY));
		robotRealSpeed
			.set((Math.hypot(this.iOdata.state.Speeds.vxMetersPerSecond, this.iOdata.state.Speeds.vyMetersPerSecond)));
		robotPose.set(new double[] {
			this.iOdata.state.Pose.getX(),
			this.iOdata.state.Pose.getY(),
			this.iOdata.state.Pose.getRotation().getRadians()});
	}

	@Override
	public void periodic() {
		RobotState.pose = iOdata.state.Pose;

		thetaController.setPID(thetaP, thetaI, thetaD);

		if (DriverStation.isDisabled()) {
			DriverStation.getAlliance().ifPresent(allianceColor -> {

				this.driveKraken.setOperatorPerspective(allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation
					: kBlueAlliancePerspectiveRotation);
			});
		}

		this.iOdata = driveKraken.update();

		UpdateTelemetry();
	}

	public Command alignToPole() {
		return runPathFinding(new Pose2d(1.05, 2.65, Rotation2d.fromDegrees(90)));
	}

	public Command runPathFinding(Pose2d target) {
		return AutoBuilder.pathfindToPose(target, constraints);
	}

	public void teleopDrive(double driveX, double driveY, double driveTheta) {
		double deadband = .15;

		driveX = Math.abs(Math.hypot(driveX, driveY)) > deadband ? driveX : 0;
		driveY = Math.abs(Math.hypot(driveX, driveY)) > deadband ? driveY : 0;
		driveTheta = Math.abs(driveTheta) > deadband ? driveTheta : 0;

		rampedX = xLimiter.calculate((driveX * Math.abs(driveX)));
		rampedY = yLimiter.calculate((driveY * Math.abs(driveY)));
		rampedTheta = rotLimiter.calculate(driveTheta * Math.abs(driveTheta));

		driveKraken.setSwerveRequest(FIELD_CENTRIC
			.withVelocityX(rampedX * DriveConfig.MAX_VELOCITY())
			.withVelocityY(rampedY * DriveConfig.MAX_VELOCITY())
			.withRotationalRate(rampedTheta * DriveConfig.MAX_ANGULAR_VELOCITY()));
	}

	public void teleopDriveSlow(double driveX, double driveY, double driveTheta) {
		// no rate limiter applied for slow mode
		driveKraken.setSwerveRequest(FIELD_CENTRIC
			.withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY()
				* slowModeMultiplier)
			.withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY()
				* slowModeMultiplier)
			.withRotationalRate((driveTheta <= 0 ? -(driveTheta * driveTheta) : (driveTheta * driveTheta))
				* DriveConfig.MAX_ANGULAR_VELOCITY() * slowModeMultiplier));
	}

	public void robotCentricTeleopDrive(double driveX, double driveY, double driveTheta) {
		driveKraken.setSwerveRequest(ROBOT_CENTRIC
			.withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
			.withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
			.withRotationalRate((driveTheta <= 0 ? -(driveTheta * driveTheta) : (driveTheta * driveTheta))
				* DriveConfig.MAX_ANGULAR_VELOCITY()));
	}

	public void lockRotation(double driveX, double driveY, Rotation2d rtarget) {
		driveKraken.setSwerveRequest(FIELD_CENTRIC
			.withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
			.withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
			.withRotationalRate(thetaController
				.calculate(iOdata.state.Pose.getRotation().getRadians(), rtarget.getRadians())));
	}

	public void facePoint(double driveX, double driveY, Pose2d point) {
		driveKraken.setSwerveRequest(FIELD_CENTRIC_FACING_ANGLE
			.withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
			.withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
			.withTargetDirection(Rotation2d.fromRadians(Math
				.atan2(point.getY() - iOdata.state.Pose.getY(), point.getX() - iOdata.state.Pose.getX()) + Math.PI)));
	}

	// Needs to be done better at some point
	public ShootOnTheFly shootOnTheFly = new ShootOnTheFly();

	public void aim(double driveX, double driveY) {
		double angle = shootOnTheFly.update(this.iOdata.state.Pose, this.iOdata.state.Speeds, true);

		driveKraken.setSwerveRequest(FIELD_CENTRIC_FACING_ANGLE
			.withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY() * .5)
			.withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY() * .5)
			.withTargetDirection(Rotation2d.fromRadians(angle)));
	}

	public void addVisionMeasurement(Pose2d calculatedPose, double timestamp, Matrix<N3, N1> stDevs) {
		this.driveKraken.updateVision(calculatedPose, timestamp, stDevs);
	}

	public Command resetPidgeon() {
		return runOnce(() -> driveKraken.resetPidgeon());
	}

	private void configurePathPlanner() {
		try {
			var config = RobotConfig.fromGUISettings();
			AutoBuilder.configure(() -> this.iOdata.state.Pose, this::setPose, () -> this.iOdata.state.Speeds, (speeds,
				feedforwards) -> driveKraken.setSwerveRequest(m_pathApplyRobotSpeeds.withSpeeds(speeds)
					.withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
					.withWheelForceFeedforwardsY(feedforwards
						.robotRelativeForcesYNewtons())), new PPHolonomicDriveController(DriveConfig
							.AUTON_TRANSLATION_PID(), DriveConfig.AUTON_ROTATION_PID()), config, () -> false, this);
		} catch (Exception ex) {
			DriverStation
				.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
		}
	}

	private void setPose(Pose2d pose) {
		this.driveKraken.seedFieldRelative(pose);
	}

	public Pose2d getPose() {
		return this.iOdata.state.Pose;
	}
}

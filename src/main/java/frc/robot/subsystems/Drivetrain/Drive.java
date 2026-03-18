package frc.robot.subsystems.Drivetrain;

import static frc.robot.subsystems.Drivetrain.DriveConstants.*;
import frc.robot.Constants;
import frc.robot.RobotState;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveRequest.ForwardPerspectiveValue;
import com.ctre.phoenix6.swerve.SwerveRequest.SwerveDriveBrake;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.VecBuilder;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.ExponentialProfile.Constraints;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotState;
import static frc.robot.subsystems.Drivetrain.DriveConstants.DEFAULT_XY_CONSTRAINTS;
import static frc.robot.subsystems.Drivetrain.DriveConstants.DriveConfig;
import static frc.robot.subsystems.Drivetrain.DriveConstants.kRotationRateLimiter;
import static frc.robot.subsystems.Drivetrain.DriveConstants.kTranslationRateLimiter;
import static frc.robot.subsystems.Drivetrain.DriveConstants.slowModeMultiplier;

import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

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

	private final SlewRateLimiter xLimiter = new SlewRateLimiter(kTranslationRateLimiter);
	private final SlewRateLimiter yLimiter = new SlewRateLimiter(kTranslationRateLimiter);
	private final SlewRateLimiter rotLimiter = new SlewRateLimiter(kRotationRateLimiter);

	private DriveKraken driveKraken;
	private DriveIOdata iOdata;

	private PathConstraints alignToLadderConstraints;

	private double rampedX;
	private double rampedY;
	private double rampedTheta;

	private PIDController translationControllerIn = new PIDController(5, 0, 0);
	private PIDController translationControllerAcross = new PIDController(5, 0, 0);
	private ProfiledPIDController translationControllerY = new ProfiledPIDController(5, 0, 0, DEFAULT_XY_CONSTRAINTS);
	private ProfiledPIDController translationControllerX = new ProfiledPIDController(5, 0, 0, DEFAULT_XY_CONSTRAINTS);

	private final SwerveDriveBrake brakeRequest = new SwerveDriveBrake();
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
	private DoubleArrayPublisher robotStatePose;

	private DoubleSubscriber poleAlignmentSub;

	private boolean isDocked = false;

	public Drive(DriveKraken driveKraken) {
		this.driveKraken = driveKraken;

		this.iOdata = driveKraken.update();

		NetworkTableInstance inst = NetworkTableInstance.getDefault();

		NetworkTable table = inst.getTable("HOT/Drive");
		NetworkTable objectDetection = inst.getTable("HOTVision/FRONT/object_detection");

		poleAlignmentSub = objectDetection.getDoubleTopic("pipe_alignment").subscribe(0.0);

		robotReqSpeed = table.getDoubleTopic("Robot Requested Speed").publish();
		robotRealSpeed = table.getDoubleTopic("Robot Real Speed").publish();
		robotPose = table.getDoubleArrayTopic("Robot Pose").publish();
		robotStatePose = table.getDoubleArrayTopic("Robot State Pose").publish();

		thetaController.enableContinuousInput(-Math.PI, Math.PI);
		translationControllerY.setTolerance(0.05, 0.1);
		translationControllerX.setTolerance(0.05, 0.1);
		configurePathPlanner();

		alignToLadderConstraints = new PathConstraints(1.5, 1.5, 2 * Math.PI, 1 * Math.PI); //.5
	}

	private void UpdateTelemetry() {
		robotReqSpeed.set(Math.hypot(FIELD_CENTRIC.VelocityX, FIELD_CENTRIC.VelocityY));
		robotRealSpeed
			.set((Math.hypot(this.iOdata.state.Speeds.vxMetersPerSecond, this.iOdata.state.Speeds.vyMetersPerSecond)));
		robotPose.set(new double[] {
			this.iOdata.state.Pose.getX(),
			this.iOdata.state.Pose.getY(),
			this.iOdata.state.Pose.getRotation().getRadians()});

		robotStatePose.set(new double[] {
			RobotState.pose.getX(),
			RobotState.pose.getY(),
			RobotState.pose.getRotation().getRadians()});
	}

	@Override
	public void periodic() {
		RobotState.pose = iOdata.state.Pose;
		RobotState.speeds = iOdata.state.Speeds;

		thetaController.setPID(thetaP, thetaI, thetaD);

		if (DriverStation.isDisabled()) {
			DriverStation.getAlliance().ifPresent(allianceColor -> {
				this.driveKraken.setOperatorPerspective(allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation
					: kBlueAlliancePerspectiveRotation);
			});
		}

		this.iOdata = driveKraken.update();

		UpdateTelemetry();

		SmartDashboard.putBoolean("Drivertrain good", isAtTargetAngle());
		SmartDashboard.putBoolean("Climb/iAmDocking", iAmDockingLeaveMeAlone);
	}

	Pose2d startPose;
	double targetDistanceMeters = -0.75; // 18 inches to ensure we hit the ladder
	double offsettoladder = 0;
	TrapezoidProfile.Constraints xconstraints = new TrapezoidProfile.Constraints(0.25, 0.75);
	TrapezoidProfile.Constraints yconstraints = new TrapezoidProfile.Constraints(0.45, 0.75);
	ProfiledPIDController xprofiledPID = new ProfiledPIDController(5, 0, 0, xconstraints);
	ProfiledPIDController yprofiledPID = new ProfiledPIDController(5, 0, 0, yconstraints);
	PIDController dockThetaController = new PIDController(2, 0, 0);
	

	public Command lineUpToLadder(){
		return Commands.runOnce(() -> {
			xprofiledPID.setTolerance(0.01);
			yprofiledPID.setTolerance(0.01);
			dockThetaController.enableContinuousInput(-Math.PI, Math.PI);
			startPose = iOdata.state.Pose;
			xprofiledPID.reset(startPose.getX());
			yprofiledPID.reset(startPose.getY());
			thetaController.reset();
			// offsettoladder = poleAlignmentSub.get() + .129;
			offsettoladder = 0.10;
		})
		.andThen(new WaitCommand(0.04)).andThen(
			Commands.run(() -> {
				SmartDashboard.putNumber("Climb/Goal X", (startPose.getX() + offsettoladder));
				double yVelocity = yprofiledPID.calculate(iOdata.state.Pose.getY(), startPose.getY() - targetDistanceMeters);
				double xVelocity = xprofiledPID.calculate(iOdata.state.Pose.getX(), startPose.getX() + offsettoladder);
				double thetaVelocity = dockThetaController.calculate(iOdata.state.Pose.getRotation().getRadians(), Math.toRadians(90));
				SmartDashboard.putNumber("Climb/xVelocity", xVelocity);
				SmartDashboard.putNumber("Climb/yVelocity", yVelocity);
				SmartDashboard.putNumber("Climb/thetaVelocity", thetaVelocity);
				driveKraken.setSwerveRequest(FIELD_CENTRIC
					.withVelocityX(xVelocity)
					.withVelocityY(yVelocity)
					.withRotationalRate(thetaVelocity)
					);
			})
		).withTimeout(5);
	}

	
	// Combines the align and dock commands into one sequence for autonomous
	public Command AutonDockToLadder() {
		return Commands.sequence(this.run(() -> alignToPole()), this.run(() -> driveToPoleCoordinate()));
	}

	public Command alignToPole() {
		return runPathFinding(new Pose2d(1.05, 2.2, Rotation2d.fromDegrees(90))) // 15.5, 5.5
			.andThen(this.run(() -> {
				driveKraken.setSwerveRequest(FIELD_CENTRIC_FACING_ANGLE
					.withTargetDirection(Rotation2d.fromDegrees(90))
					.withVelocityY(0.1)); //trying to drive across the field slightly to set swerve to Y dir
			}
			).withTimeout(1));
	}
	
	double xTarget = 0;
	double yTarget = 0;
	double rotationOffset = 0;
	double targetAngleRad = Math.toRadians(90);
	double currentAngleRad = 0;
	double angleErrorRad = 0;
	private boolean iAmDockingLeaveMeAlone = false;
	
public Command driveToPoleCoordinate() {
    return Commands.defer(() -> {
        Pose2d currentPose = iOdata.state.Pose;
        currentAngleRad = currentPose.getRotation().getRadians();
        angleErrorRad = targetAngleRad - currentAngleRad;
        rotationOffset = Math.abs(targetDistanceMeters) * Math.sin(angleErrorRad); 
        offsettoladder = poleAlignmentSub.get() + 0.15 - rotationOffset;
        double targetX = currentPose.getX() + offsettoladder;
        double targetY = currentPose.getY() - targetDistanceMeters;
        Rotation2d targetRotation = Rotation2d.fromDegrees(90);
		double rearAxleOffsetMeters = -0.3175; // Distance from center to back wheels
        double sweepFrequency = 3.0;        // Shakes per second
        double sweepMagnitudeRad = 0.75;     // How wide the "arc" is (in Radians)
        double forwardMps = 0.4;           // Constant push from back motors
		iAmDockingLeaveMeAlone = true;

		Command wiggle = Commands.run(() -> {
            double rotationWiggle = Math.sin(Timer.getFPGATimestamp() * 2 * Math.PI * sweepFrequency) * sweepMagnitudeRad;

            driveKraken.setSwerveRequest(ROBOT_CENTRIC
                .withVelocityX(forwardMps)
                .withVelocityY(0)
                .withRotationalRate(rotationWiggle)
                .withCenterOfRotation(new Translation2d(rearAxleOffsetMeters, 0))
            );
        }, this).withTimeout(1.0);

        return runPathFinding(new Pose2d(targetX, targetY, targetRotation))
               .andThen(wiggle).andThen(() -> iAmDockingLeaveMeAlone = false);
               
    }, Set.of(this));
}

	public Command runPathFinding(Pose2d target) {
		return AutoBuilder.pathfindToPose(target, alignToLadderConstraints);
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

	public Command xLockCommand() {
		return this.run(() -> driveKraken.setSwerveRequest(brakeRequest));
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

	public void aim(double driveX, double driveY) {
		driveKraken.setSwerveRequest(FIELD_CENTRIC_FACING_ANGLE
			.withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY() * .25)
			.withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY() * .25)
			.withTargetDirection(Rotation2d.fromRadians(RobotState.turretAngle)));
	}

	// align to fixed Y coordinate on field using Y translation controller
	public Command driveToTrenchY(DoubleSupplier driveX) {
		final Rotation2d[] targetRotation = new Rotation2d[1];

		return this.runOnce(() -> {
			double targetY = (iOdata.state.Pose.getY() < 4.035) ? 0.637 : 7.42;
			double currentDegrees = iOdata.state.Pose.getRotation().getDegrees();
			translationControllerY.reset(iOdata.state.Pose.getY(), iOdata.state.Speeds.vyMetersPerSecond);
			translationControllerY.setGoal(targetY);
			targetRotation[0] = (Math.abs(currentDegrees - 90) < Math.abs(currentDegrees + 90))
				? Rotation2d.fromDegrees(90)
				: Rotation2d.fromDegrees(-90);
		})
			.andThen(this.run(() -> {
				double yOutput = translationControllerY.calculate(iOdata.state.Pose.getY());

				if (translationControllerY.atGoal()) {
					yOutput = 0;
				}

				driveKraken.setSwerveRequest(FIELD_CENTRIC
					.withVelocityX(driveX.getAsDouble() * DriveConfig.MAX_VELOCITY())
					.withVelocityY(-yOutput)
					.withRotationalRate(thetaController
						.calculate(iOdata.state.Pose.getRotation().getRadians(), targetRotation[0].getRadians())));
			}));
	}

	public void addVisionMeasurement(Pose2d calculatedPose, double timestamp, Matrix<N3, N1> stDevs) {
		if (iAmDockingLeaveMeAlone) {
			stDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
		}
		this.driveKraken.updateVision(calculatedPose, timestamp, stDevs);
	}

	public Command resetPidgeon() {
		return runOnce(() -> driveKraken.resetPidgeon());
	}

	public Command resetControllers() {
		return runOnce(() -> {
			translationControllerX.reset(iOdata.state.Pose.getX(), iOdata.state.Speeds.vxMetersPerSecond);
			translationControllerY.reset(iOdata.state.Pose.getY(), iOdata.state.Speeds.vyMetersPerSecond);
			translationControllerIn.reset();
			translationControllerAcross.reset();
		});
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

	public boolean isAtTargetAngle() {
		if (RobotState.color == Alliance.Red) {
			return Math.abs(iOdata.state.Pose.getRotation().getDegrees()
				- (Math.toDegrees(RobotState.turretAngle - Math.PI))) < 5;
		} else {
			return Math
				.abs(iOdata.state.Pose.getRotation().getDegrees() - (Math.toDegrees(RobotState.turretAngle))) < 5;
		}
	}

	public boolean isAtTargetAngleAuton(){
		if (RobotState.color == Alliance.Red) {
			return Math.abs(iOdata.state.Pose.getRotation().getDegrees() - (Math.toDegrees(RobotState.turretAngle - Math.PI) )) < 5;
		} else {			
			return Math.abs(iOdata.state.Pose.getRotation().getDegrees() - (Math.toDegrees(RobotState.turretAngle) )) < 5;
		}
	}

	public Command driveTowardsClimberSide(double speed) {
		return run(() -> {
			driveKraken.setSwerveRequest(ROBOT_CENTRIC
			.withVelocityY(speed));
		}).finallyDo(() -> driveKraken.setSwerveRequest(ROBOT_CENTRIC.withVelocityY(0)));
	}
}

package frc.robot.subsystems.Drivetrain;

import static edu.wpi.first.units.Units.*;

import java.util.Set;
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
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotState;
import frc.robot.subsystems.Drivetrain.constants.DrivetrainConstants;
import frc.robot.subsystems.Drivetrain.constants.DrivetrainConstants.TunerSwerveDrivetrain;
import static frc.robot.subsystems.Drivetrain.constants.DrivetrainConstants.left_blue_ladder_angle_deg;
import static frc.robot.subsystems.Drivetrain.constants.DrivetrainConstants.right_blue_ladder_angle_deg;
import static frc.robot.subsystems.Drivetrain.constants.DrivetrainConstants.left_red_ladder_angle_deg;
import static frc.robot.subsystems.Drivetrain.constants.DrivetrainConstants.right_red_ladder_angle_deg;

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

	private final SwerveRequest.RobotCentric ROBOT_CENTRIC = new SwerveRequest.RobotCentric();

    private final DrivetrainTelemetry logger = new DrivetrainTelemetry(MaxSpeed);
	
	private DoubleSubscriber poleAlignmentSub;
	private DoubleSubscriber poleDistanceSub;
	private PathConstraints alignToLadderConstraints = new PathConstraints(0.75, 1.0, 2 * Math.PI, 1 * Math.PI); //.5;


    public Drivetrain(SwerveDrivetrainConstants drivetrainConstants, SwerveModuleConstants<?, ?, ?>... modules) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configureAutoBuilder();

		registerTelemetry(logger::telemeterize);
		
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTable objectDetection = inst.getTable("HOTVision/LEFT/object_detection");
		poleAlignmentSub = objectDetection.getDoubleTopic("pipe_alignment").subscribe(0.0);
		poleDistanceSub = objectDetection.getDoubleTopic("pipe_distance").subscribe(0.0);

		SmartDashboard.putBoolean("climb/Use Cam Dist", false);
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
			.withTargetDirection(Rotation2d.fromRadians(RobotState.turretAngle)));
	}

    public Command applyRequest(Supplier<SwerveRequest> request) {
        return run(() -> this.setControl(request.get()));
    }

    @Override
    public void periodic() {
        if (DriverStation.isDisabled()) {
			setOperatorPerspectiveForward(RobotState.color == Alliance.Red
                        ? Rotation2d.k180deg
                        : Rotation2d.kZero);
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

	public Command driveTowardsClimberSide(double speed) {
		return run(() -> {
			setControl(ROBOT_CENTRIC
			.withVelocityY(speed));
		}).finallyDo(() -> setControl(ROBOT_CENTRIC.withVelocityY(0)));
	}

	public boolean isAtTargetAngle(){
		if (RobotState.color == Alliance.Red) {
			return Math.abs(getState().Pose.getRotation().getDegrees() - (Math.toDegrees(RobotState.turretAngle - Math.PI) )) < 5;
		} else {			
			return Math.abs(getState().Pose.getRotation().getDegrees() - (Math.toDegrees(RobotState.turretAngle) )) < 5;
		}
	}

final double CLIMBER_CHASSIS_OFFSET = 90.0; 	// Climber on front
final double FIELD_WIDTH_MIDPOINT = 4.035;   // Midpoint of the 8.069m width
double TARGET_DISTANCE = 0.6;          // The 0.7m gap from Auto drop-off
public boolean iAmDockingLeaveMeAlone = false;

public Command driveToPoleCoordinate_autoflip() {
    return Commands.defer(() -> {
        Pose2d currentPose = getState().Pose;
        boolean aboveYMidpoint = currentPose.getY() > FIELD_WIDTH_MIDPOINT;
        double approachDirection = aboveYMidpoint ? -1.0 : 1.0;
        
		double baseApproachAngle = 0.0;
        // Desired Chassis Rotation
		if (DriverStation.getAlliance().get() == Alliance.Red) {
       		baseApproachAngle = (aboveYMidpoint ? -90.0 + left_blue_ladder_angle_deg : 90.0 + right_blue_ladder_angle_deg);
		} else {
			baseApproachAngle = (aboveYMidpoint ? -90.0 + right_red_ladder_angle_deg : 90.0 + left_red_ladder_angle_deg);
		}
        Rotation2d targetRotation = Rotation2d.fromDegrees(baseApproachAngle - CLIMBER_CHASSIS_OFFSET);

        // Chassis Rotation Error
        double currentAngleRad = currentPose.getRotation().getRadians();
        double angleErrorRad = targetRotation.getRadians() - currentAngleRad;
        
		TARGET_DISTANCE = MathUtil.clamp(poleDistanceSub.get(), 0.5, 1.0);
		

        // rotationXOffset (in meters) accounts for the chassis rotation impact on camera offset
        double rotationXOffset = 0.5 * Math.abs(TARGET_DISTANCE) * Math.sin(angleErrorRad); 
		
		// targetY: Move 0.7m toward middle of field
        double targetY = currentPose.getY() + (TARGET_DISTANCE * approachDirection);
        
		// targetX: Adjust current X based on camera (poleAlignmentSub) and rotation swing
		double camOffset = 0.11; // Default camera offset in meters

        double targetX = currentPose.getX() + (MathUtil.clamp(poleAlignmentSub.get(), -0.3, 0.3) + camOffset + rotationXOffset) * approachDirection;

        Pose2d finalTarget = new Pose2d(targetX, targetY, targetRotation);
		
		SmartDashboard.putNumber("climb/cmd_rotationXOffset", rotationXOffset);
		SmartDashboard.putNumber("climb/cmd_camOffset", camOffset);
		SmartDashboard.putNumber("climb/cmd_currPoseX", currentPose.getX());
		SmartDashboard.putNumber("climb/cmd_clmpdPoleAlign", (MathUtil.clamp(poleAlignmentSub.get(), -0.3, 0.3)) );
		SmartDashboard.putNumber("climb/cmd_TargetDistance", TARGET_DISTANCE);
        
		// Telemetry & Vision off while docking
        logger.dockingTargetPub.set(finalTarget);
        iAmDockingLeaveMeAlone = true;

        //Wiggle
        double pushXvel = 0.3; // 0.4 meters per second forward

        Command wiggle = Commands.run(() -> {
            double rotationWiggle = Math.sin(Timer.getFPGATimestamp() * 2 * Math.PI * 3.0) * 0.75;
            setControl(ROBOT_CENTRIC
                .withVelocityY(pushXvel)
                .withVelocityX(0)
                .withRotationalRate(rotationWiggle)
                .withCenterOfRotation(new Translation2d(-0.3175, 0)) // Meters
            );
        }, this).withTimeout(1.5);

		Command drive = Commands.run(() -> {
            setControl(ROBOT_CENTRIC
                .withVelocityY(1.0) //climber is on the side of the robot to the left
                .withVelocityX(0)
                .withRotationalRate(0)
            );
        }, this).withTimeout(0.5);



        return Commands.sequence(
			runPathFinding(finalTarget),
                wiggle,
				drive,
                runOnce(() -> iAmDockingLeaveMeAlone = false))
			.onlyIf(() -> Math.abs(poleAlignmentSub.get()) < 0.3 && poleDistanceSub.get() > 0.5 && poleDistanceSub.get() < 1.0);
                
    }, Set.of(this));
}

	public Command angleSwerveWheels() {
		return Commands.run(() -> {
            setControl(ROBOT_CENTRIC
                .withVelocityY(0.1) 
                .withVelocityX(0)
                .withRotationalRate(0)
            );
        }, this).withTimeout(0.4);
	}

	public Command runPathFinding(Pose2d target) {
		return AutoBuilder.pathfindToPose(target, alignToLadderConstraints);
	}
}
package frc.robot.subsystems.Drivetrain;

import static frc.robot.subsystems.Drivetrain.DriveConstants.*;
import java.util.List;
import java.util.Set;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;    
import frc.robot.subsystems.Drivetrain.ShootingWhileMoving;

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

    private final SlewRateLimiter xLimiter = new SlewRateLimiter(kTranslationRateLimiter);
    private final SlewRateLimiter yLimiter = new SlewRateLimiter(kTranslationRateLimiter);
    private final SlewRateLimiter rotLimiter = new SlewRateLimiter(kRotationRateLimiter);
       
    private DriveKraken driveKraken;
    private DriveIOdata iOdata;

    private ShuffleboardTab driveTab;
    private GenericEntry speedEntry;
    private GenericEntry poseEntry;
    private PathConstraints constraints;
    private List<Waypoint> waypoints;

    private PathPlannerPath path;

    double rampedX;
    double rampedY;
    double rampedTheta;

    private PIDController thetaController = new PIDController(10, 0, 0.2);
    private PIDController translationControllerIn = new PIDController(5, 0, 0);
    private PIDController translationControllerAcross = new PIDController(5, 0, 0);
    private ProfiledPIDController translationControllerY = new ProfiledPIDController(5, 0, 0, DEFAULT_XY_CONSTRAINTS);
    private ProfiledPIDController translationControllerX = new ProfiledPIDController(5, 0, 0, DEFAULT_XY_CONSTRAINTS);

    private final SwerveRequest.FieldCentric FIELD_CENTRIC = new SwerveRequest.FieldCentric()
        .withDeadband(0.2).withRotationalDeadband(0.0)
        .withDriveRequestType(DriveRequestType.Velocity).withDesaturateWheelSpeeds(false);
    private final SwerveRequest.RobotCentric ROBOT_CENTRIC = new SwerveRequest.RobotCentric();
    private final SwerveRequest.ApplyRobotSpeeds m_pathApplyRobotSpeeds = new SwerveRequest.ApplyRobotSpeeds().withDriveRequestType(DriveRequestType.Velocity);

    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;

    public Drive(DriveKraken driveKraken) { 
        this.driveKraken = driveKraken;
        this.iOdata = driveKraken.update();

        translationControllerX.setTolerance(auto_align_tolerance);
        translationControllerY.setTolerance(auto_align_tolerance);

        translationControllerIn.setTolerance(auto_align_tolerance);
        translationControllerAcross.setTolerance(auto_align_tolerance);

        Rotation2d.fromDegrees(0);

        driveTab = Shuffleboard.getTab("Drive");
        speedEntry = driveTab.add("Speed", 0.0).getEntry();
        poseEntry = driveTab.add("Pose", new Double[] {0.0, 0.0, 0.0}).getEntry();

        double driveBaseRadius = 0;
        for (var moduleLocation : this.iOdata.m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        configurePathPlanner();

        constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
    }

    @Override
    public void periodic() {
        if (!(DriverStation.isTeleopEnabled()) || Math.abs(iOdata.pigeon.getX()) > 0.2 || Math.abs(iOdata.pigeon.getY()) > 0.2) {
            iOdata.state.Pose.getRotation();
        }

        if (DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                this.driveKraken.setOperatorPerspective(
                    allianceColor == Alliance.Red ? kRedAlliancePerspectiveRotation : kBlueAlliancePerspectiveRotation
                ); 
            });
        }
		
        this.iOdata = driveKraken.update();
        if (this.iOdata.state.Speeds != null) {
            speedEntry.setDouble(Math.hypot(
                this.iOdata.state.Speeds.vxMetersPerSecond,
                this.iOdata.state.Speeds.vyMetersPerSecond) / 5.0);

            SmartDashboard.putNumber("act Speed", (Math.hypot(
                this.iOdata.state.Speeds.vxMetersPerSecond,
                this.iOdata.state.Speeds.vyMetersPerSecond) ));

            SmartDashboard.putNumber("req Speed", (Math.hypot(
                FIELD_CENTRIC.VelocityX,
                FIELD_CENTRIC.VelocityY) ));
        }
        
        if (this.iOdata.state.Pose != null) {
            poseEntry.setDoubleArray(new Double[]{
                this.iOdata.state.Pose.getX(), 
                this.iOdata.state.Pose.getY(), 
                this.iOdata.state.Pose.getRotation().getRadians()});
        }

    }
            
    public boolean drivetrainAtTarget() {
        if (path != null) {
            System.out.println("here");
            return Math.abs(
                this.iOdata.state.Pose.getTranslation().getDistance(
                    path.getPathPoses().get(path.getPathPoses().size() - 1).getTranslation())) 
                    < OTF_end_tolerance;
        }
        return false;
    }

    public Command runOnTheFly() {
        return Commands.sequence(
            Commands.runOnce(() -> {
                Pose2d target = new Pose2d(2.4, 7.2, Rotation2d.fromDegrees(0));
                waypoints = PathPlannerPath.waypointsFromPoses(
                    iOdata.state.Pose,
                    target
                );
                path = new PathPlannerPath(
                    waypoints,
                    constraints,
                    null,
                    new GoalEndState(0, target.getRotation())
                );
                path.preventFlipping = true;
            }),
            // Defer path following until after the path is created so it can run in-sequence.
            Commands.defer(
                () -> AutoBuilder.followPath(path).until(this::drivetrainAtTarget),
                Set.of(this) 
            )
        );
    }

    public Command setCoast() {
        return runOnce(() -> driveKraken.setNeutralMode(NeutralModeValue.Coast));
    }

    public Command setBrake() {
        return runOnce(() -> driveKraken.setNeutralMode(NeutralModeValue.Brake));
    }

    public void teleopDrive(double driveX, double driveY, double driveTheta)  {
        rampedX = xLimiter.calculate(driveX); 
        rampedY = yLimiter.calculate(driveY);    
        rampedTheta = rotLimiter.calculate(driveTheta); 

        driveKraken.setSwerveRequest(FIELD_CENTRIC
            .withVelocityX((rampedX <= 0 ? -(rampedX * rampedX) : (rampedX * rampedX)) * DriveConfig.MAX_VELOCITY())
            .withVelocityY((rampedY <= 0 ? -(rampedY * rampedY) : (rampedY * rampedY)) * DriveConfig.MAX_VELOCITY())
            .withRotationalRate((rampedTheta <= 0 ? -(rampedTheta * rampedTheta) : (rampedTheta * rampedTheta)) * DriveConfig.MAX_ANGULAR_VELOCITY())
        );
    }

    public void teleopDriveSlow(double driveX, double driveY, double driveTheta)  {
        //no rate limiter applied for slow mode
        driveKraken.setSwerveRequest(FIELD_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY() * slowModeMultiplier)
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY() * slowModeMultiplier)
            .withRotationalRate((driveTheta <= 0 ? -(driveTheta * driveTheta) : (driveTheta * driveTheta)) * DriveConfig.MAX_ANGULAR_VELOCITY() * slowModeMultiplier)
        );
      }

    public void robotCentricTeleopDrive(double driveX, double driveY, double driveTheta)  {
        driveKraken.setSwerveRequest(ROBOT_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
            .withRotationalRate((driveTheta <= 0 ? -(driveTheta * driveTheta) : (driveTheta * driveTheta)) * DriveConfig.MAX_ANGULAR_VELOCITY())
        );
    }
 
     public void lockRotation(double driveX, double driveY, Rotation2d rtarget) {
        driveKraken.setSwerveRequest(FIELD_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
            .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(), rtarget.getRadians()))
        );
     }

     public ShootingWhileMoving ShootingOnTarget = new ShootingWhileMoving(); 
 
     public void facePoint(double driveX, double driveY, Pose2d point) {
          
        double angle = ShootingOnTarget.update(this.iOdata.state.Pose, this.iOdata.state.Speeds, true);

        driveKraken.setSwerveRequest(FIELD_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
            .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(),
            angle + Math.PI)
            //Math.atan2(point.getY() - iOdata.state.Pose.getY(), point.getX() - iOdata.state.Pose.getX()) + Math.PI))
        ));        
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
            AutoBuilder.configure(
                () -> this.iOdata.state.Pose,
                this::setPose,
                () -> this.iOdata.state.Speeds,
                (speeds, feedforwards) -> driveKraken.setSwerveRequest(
                    m_pathApplyRobotSpeeds.withSpeeds(speeds)
                        .withWheelForceFeedforwardsX(feedforwards.robotRelativeForcesXNewtons())
                        .withWheelForceFeedforwardsY(feedforwards.robotRelativeForcesYNewtons())
                ),
                new PPHolonomicDriveController(
                    DriveConfig.AUTON_TRANSLATION_PID(),
                    DriveConfig.AUTON_ROTATION_PID()
                ),
                config,
                () -> false,
                this
            );
        } catch (Exception ex) {
            DriverStation.reportError("Failed to load PathPlanner config and configure AutoBuilder", ex.getStackTrace());
        }
    }

    private void setPose(Pose2d pose) {
        this.driveKraken.seedFieldRelative(pose);
    }
}


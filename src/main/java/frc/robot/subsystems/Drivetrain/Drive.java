package frc.robot.subsystems.Drivetrain;

import static frc.robot.subsystems.Drivetrain.DriveConstants.*;
import frc.robot.Constants;
import frc.robot.RobotState;

import java.util.Arrays;
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
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Robot;
import frc.util.FuelSim;
import static edu.wpi.first.units.Units.*;
  

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

private final PIDController thetaController =
    new PIDController(thetaP, thetaI, thetaD);

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
    private List<Waypoint> waypoints;

    private PathPlannerPath path;

    private double rampedX;
    private double rampedY;
    private double rampedTheta;


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

    private DoublePublisher robotReqSpeed;
    private DoublePublisher robotRealSpeed;
    private DoubleArrayPublisher robotPose;

    public record ShooterState(double velocityMps, double angleDeg) {}
	private int lastScoreCount = 0;
	private double measuredTOF = 0;
	private double lastLaunchTime = 0;
	
    public Drive(DriveKraken driveKraken) { 

    if (!Constants.compMode) {
       
        tuningTable =
        NetworkTableInstance.getDefault().getTable("Tuning/Drive");

    thetaP_pub = tuningTable.getDoubleTopic("Theta kP").publish();
    thetaI_pub = tuningTable.getDoubleTopic("Theta kI").publish();
    thetaD_pub = tuningTable.getDoubleTopic("Theta kD").publish();

    thetaP_pub.set(thetaP);
    thetaI_pub.set(thetaI);
    thetaD_pub.set(thetaD);

    thetaP_sub = tuningTable.getDoubleTopic("Theta kP").subscribe(thetaP);
    thetaI_sub = tuningTable.getDoubleTopic("Theta kI").subscribe(thetaI);
    thetaD_sub = tuningTable.getDoubleTopic("Theta kD").subscribe(thetaD);

    }

        this.driveKraken = driveKraken;

        this.iOdata = driveKraken.update();

        translationControllerX.setTolerance(auto_align_tolerance);
        translationControllerY.setTolerance(auto_align_tolerance);

        translationControllerIn.setTolerance(auto_align_tolerance);
        translationControllerAcross.setTolerance(auto_align_tolerance);

        Rotation2d.fromDegrees(0);
 
         NetworkTableInstance inst = NetworkTableInstance.getDefault();

        NetworkTable table = inst.getTable("HOT/Drive");

        robotReqSpeed = table.getDoubleTopic("Robot Requested Speed").publish();
        robotRealSpeed = table.getDoubleTopic("Robot Real Speed").publish();
        robotPose = table.getDoubleArrayTopic("Robot Pose").publish();

    
        double driveBaseRadius = 0;
        for (var moduleLocation : this.iOdata.m_moduleLocations) {
            driveBaseRadius = Math.max(driveBaseRadius, moduleLocation.getNorm());
        }

        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        configurePathPlanner();

        constraints = new PathConstraints(3.0, 3.0, 2 * Math.PI, 4 * Math.PI); // The constraints for this path.
		SmartDashboard.setDefaultBoolean("Tuning/ModeActive", false);
    	SmartDashboard.setDefaultNumber("Tuning/ShotVelocity", 5.0);
    	SmartDashboard.setDefaultNumber("Tuning/ShotAngle", 40.0);
    }

    private void UpdateTelemetry() {
        robotReqSpeed.set(Math.hypot(
                FIELD_CENTRIC.VelocityX,
                FIELD_CENTRIC.VelocityY) );
        robotRealSpeed.set((Math.hypot(
                this.iOdata.state.Speeds.vxMetersPerSecond,
                this.iOdata.state.Speeds.vyMetersPerSecond) ));
        robotPose.set(new double[]{
                this.iOdata.state.Pose.getX(), 
                this.iOdata.state.Pose.getY(), 
                this.iOdata.state.Pose.getRotation().getRadians()});        
    }

    @Override
    public void periodic() {
		
		RobotState.pose = iOdata.state.Pose;

		if (!Constants.compMode) {
			thetaP = thetaP_sub.get();
			thetaI = thetaI_sub.get();
			thetaD = thetaD_sub.get();

			thetaP_pub.set(thetaP);
			thetaI_pub.set(thetaI);
			thetaD_pub.set(thetaD);
		}

		thetaController.setPID(thetaP, thetaI, thetaD);



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

        UpdateTelemetry();
		updateScoreTelemetry();
    }
            
    public boolean drivetrainAtTarget() {
        if (path != null) {
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

    public void facePoint(double driveX, double driveY, Pose2d point) {
        driveKraken.setSwerveRequest(FIELD_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
            .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(),
            Math.atan2(point.getY() - iOdata.state.Pose.getY(), point.getX() - iOdata.state.Pose.getX()) + Math.PI)));
    }

    // Needs to be done better at some point
    public ShootOnTheFly shootOnTheFly = new ShootOnTheFly(); 

    public void aim(double driveX, double driveY){
        double angle = shootOnTheFly.update(this.iOdata.state.Pose, this.iOdata.state.Speeds, true);

        driveKraken.setSwerveRequest(FIELD_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
            .withVelocityY((driveY <= 0 ? -(driveY * driveY) : (driveY * driveY)) * DriveConfig.MAX_VELOCITY())
            .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(), angle)
        ));

        
    }
    //align to fixed Y coordinate on field using Y translation controller
    public void fixedY(double driveX, Rotation2d rtarget) {
         double dynamicY = (iOdata.state.Pose.getY() < 4.035) ? 0.637 : 7.42;
        double yOutput = translationControllerY.calculate(iOdata.state.Pose.getY(), dynamicY);
        driveKraken.setSwerveRequest(FIELD_CENTRIC
            .withVelocityX((driveX <= 0 ? -(driveX * driveX) : (driveX * driveX)) * DriveConfig.MAX_VELOCITY())
            .withVelocityY(!translationControllerY.atGoal() ? -yOutput : 0.0)
            .withRotationalRate(thetaController.calculate(iOdata.state.Pose.getRotation().getRadians(), rtarget.getRadians()))
        );
        SmartDashboard.putNumber("Target Y", dynamicY);
        SmartDashboard.putNumber("Current Y", iOdata.state.Pose.getY());
        SmartDashboard.putNumber("PID Y Output", yOutput);
    }

    public void addVisionMeasurement(Pose2d calculatedPose, double timestamp, Matrix<N3, N1> stDevs) {
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

	// This method determines the target shooter velocity and angle for simulation.
	public ShooterState getSimShotTarget() {
		// 1. Check a "Tuning Mode" toggle on the Dashboard
		// If this is true, use sliders. If false, use the Lookup Table.
		boolean tuningMode = SmartDashboard.getBoolean("Tuning/ModeActive", false);
	
		if (tuningMode) {
			double vel = SmartDashboard.getNumber("Tuning/ShotVelocity", 5.0);
			double angle = SmartDashboard.getNumber("Tuning/ShotAngle", 40.0);
			return new ShooterState(vel, angle);
		} else {
			// 2. Auto-Mode: Get distance to the Red Hub
			Translation2d redHub = new Translation2d(11.91, 4.03);
			double dist = iOdata.state.Pose.getTranslation().getDistance(redHub);
	
			// 3. Lookup values from your ShootOnTheFly maps
			double vel = ShootOnTheFly.getFlywheelSpeed(dist);
			double angle = ShootOnTheFly.getHoodAngle(dist).getDegrees();
			SmartDashboard.putNumber("Shooter/ReferenceDistance", dist);
        	SmartDashboard.putNumber("Shooter/ReferenceVelocity", vel);
        	SmartDashboard.putNumber("Shooter/ReferenceAngle", angle);
			return new ShooterState(vel, angle);
		}
	}

	// This method sends a launch command to the FuelSim with the appropriate parameters.
	public void launchSimFuel(FuelSim fuelSim) {
		if (Robot.isSimulation() && fuelSim != null) {
			// 1. Start the stopwatch
			lastLaunchTime = Timer.getFPGATimestamp();

			ShooterState state = getSimShotTarget();
			
			// 2. Launch using your preferred 180-degree heading
			fuelSim.launchFuel(
				MetersPerSecond.of(state.velocityMps()),
				Degrees.of(state.angleDeg()),
				Degrees.of(180), 
				Meters.of(0.3)   
			);

			System.out.println("Shot Fired! Time logged.");
		}
	}

	// This method updates the Dashboard with the current scores from the FuelSim.
	public void updateScoreTelemetry() {
		// Access the static Hub instances inside FuelSim
		int blueScore = FuelSim.Hub.BLUE_HUB.getScore();
		int redScore = FuelSim.Hub.RED_HUB.getScore();
	
		SmartDashboard.putNumber("Sim/Blue Hub Score", blueScore);
		SmartDashboard.putNumber("Sim/Red Hub Score", redScore);
		
	}


	public void resetPose(Pose2d pose) {
		// In Phoenix 6, this "seeds" the odometry with a new position
		this.driveKraken.seedFieldRelative(pose);
		
		// For Simulation: This updates the 'ghost' robot in the sim environment
		if (Robot.isSimulation()) {
			// We look for the pose inside your iOdata to keep it in sync
			this.iOdata.state.Pose = pose;
		}
	}

	// This command runs the entire characterization sequence for the shooter in sim, allowing you to collect data at various distances.
	public Command shooterCharacterizer(CommandXboxController driver, FuelSim fuelSim) {
		double[] testDistances = {1.34, 2.17, 3.15, 4.09, 5.57};
		StringBuilder finalTable = new StringBuilder(); // Storage for results
		
		var sequence = Commands.runOnce(() -> {
			System.out.println("--- TUNING STARTED ---");
			finalTable.append("\n/* --- GENERATED LOOKUP TABLE --- */\n");
		});
	
		for (double dist : testDistances) {
			sequence = sequence.andThen(
				Commands.sequence(
					// 1. Teleport
					Commands.runOnce(() -> {
						Translation2d redGoal = new Translation2d(11.91, 4.03); 
						this.resetPose(new Pose2d(redGoal.getX() + (dist * 0.707), redGoal.getY() + (dist * 0.707), redGoal.minus(new Translation2d(redGoal.getX() + (dist * 0.707), redGoal.getY() + (dist * 0.707))).getAngle()));
						System.out.println("\n>>> CURRENT DISTANCE: " + dist + "m (Press X to shoot, BACK for next)");
					}),
					
					// 2. The Tuning Window (Inside the for-loop)
					Commands.run(() -> {
						// A. Fire the ball
						if (driver.getHID().getXButtonPressed()) {
							lastLaunchTime = Timer.getFPGATimestamp();
							this.launchSimFuel(fuelSim);
						}

						// B. Watch for score increase to measure TOF
						int currentScore = FuelSim.Hub.RED_HUB.getScore();
						if (currentScore > lastScoreCount) {
							measuredTOF = Timer.getFPGATimestamp() - lastLaunchTime;
							lastScoreCount = currentScore; // Sync the count
							System.out.println("Score Detected! Measured TOF: " + measuredTOF);
						}
					})
					.until(driver.back()),
					// 3. Store Results (Inside the for-loop)
					Commands.runOnce(() -> {
						double v = SmartDashboard.getNumber("Tuning/ShotVelocity", 5.0);
						double a = SmartDashboard.getNumber("Tuning/ShotAngle", 40.0);
						
						// Fallback: If we didn't score this round, calculate the math version
						if (measuredTOF <= 0) {
							double horizontalVel = v * Math.cos(Math.toRadians(a));
							measuredTOF = (horizontalVel > 0) ? dist / horizontalVel : 0;
						}

						finalTable.append(String.format("// Data for %.2fm\n", dist));
						finalTable.append(String.format("shotFlywheelSpeedMap.put(%.2f, %.2f);\n", dist, v));
						finalTable.append(String.format("shotHoodAngleMap.put(%.2f, Rotation2d.fromDegrees(%.2f));\n", dist, a));
						finalTable.append(String.format("timeOfFlightMap.put(%.2f, %.2f); // Measured\n\n", dist, measuredTOF));
						
						// Reset measuredTOF for the next distance in the loop
						measuredTOF = 0;
					}),
					Commands.waitSeconds(0.5)
				)
			);
		}
	
		// 4. The Grand Finale: Print everything at once!
		return sequence.finallyDo(() -> {
			System.out.println("\n\n" + finalTable.toString());
			System.out.println("--- ALL DISTANCES COMPLETE ---");
		});
	}

    public Pose2d getPose() {
        return this.iOdata.state.Pose;
    }

    public ChassisSpeeds getChassisSpeeds() {
        return new ChassisSpeeds(
            this.iOdata.state.Speeds.vxMetersPerSecond,
            this.iOdata.state.Speeds.vyMetersPerSecond,
            this.iOdata.state.Speeds.omegaRadiansPerSecond
        );
    }
}

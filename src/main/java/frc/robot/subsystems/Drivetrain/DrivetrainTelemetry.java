package frc.robot.subsystems.Drivetrain;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.RobotState;
import frc.util.HubTracker;
import edu.wpi.first.wpilibj.DriverStation;
import com.ctre.phoenix6.BaseStatusSignal;
import edu.wpi.first.units.measure.Current;


public class DrivetrainTelemetry {
    private final double MaxSpeed;
	
	private final StatusSignal<Current>[] driveCurrentSignals = new StatusSignal[4];
	
	private final StatusSignal<Current>[] steerCurrentSignals = new StatusSignal[4];
	private final DoublePublisher driveTotalCurrent;

    /**
     * Construct a telemetry object, with the specified max speed of the robot
     * 
     * @param maxSpeed Maximum speed in meters per second
     */
    public DrivetrainTelemetry(double maxSpeed, SwerveDrivetrain drivetrain) {
        MaxSpeed = maxSpeed;
		driveTotalCurrent = driveStateTable.getDoubleTopic("TotalCurrent").publish();

        SignalLogger.start();

        /* Set up the module state Mechanism2d telemetry */
        for (int i = 0; i < 4; ++i) {
			driveCurrentSignals[i] = drivetrain.getModule(i).getDriveMotor().getSupplyCurrent();
			steerCurrentSignals[i] = drivetrain.getModule(i).getSteerMotor().getSupplyCurrent();

			driveCurrentSignals[i].setUpdateFrequency(50);
			steerCurrentSignals[i].setUpdateFrequency(50);
		
        }
    }

    /* What to publish over networktables for telemetry */
    private final NetworkTableInstance inst = NetworkTableInstance.getDefault();

    /* Robot swerve drive state */
    private final NetworkTable driveStateTable = inst.getTable("HOT/Drive");
    private final StructPublisher<Pose2d> drivePose = driveStateTable.getStructTopic("Pose", Pose2d.struct).publish();
    private final StructPublisher<ChassisSpeeds> driveSpeeds = driveStateTable.getStructTopic("Speeds", ChassisSpeeds.struct).publish();
    private final DoublePublisher driveTimestamp = driveStateTable.getDoubleTopic("Timestamp").publish();
    private final DoublePublisher driveOdometryFrequency = driveStateTable.getDoubleTopic("OdometryFrequency").publish();
	public final StructPublisher<Pose2d> dockingTargetPub = driveStateTable.getStructTopic("Docking Target", Pose2d.struct).publish();
	public final DoublePublisher hubTimeRemaining = driveStateTable.getDoubleTopic("Hub Time Remaining").publish(); 
	public final BooleanPublisher hubActive = driveStateTable.getBooleanTopic("Hub Active").publish();
	public final DoublePublisher matchTime = driveStateTable.getDoubleTopic("Match Time").publish();

    /** Accept the swerve drive state and telemeterize it to SmartDashboard and SignalLogger. */
    public void telemeterize(SwerveDriveState state) {
        /* Telemeterize the swerve drive state */
        drivePose.set(state.Pose);
        driveSpeeds.set(state.Speeds);
        driveTimestamp.set(state.Timestamp);
        driveOdometryFrequency.set(1.0 / state.OdometryPeriod);

        /* Also write to log file */
        SignalLogger.writeStruct("DriveState/Pose", Pose2d.struct, state.Pose);
        SignalLogger.writeStruct("DriveState/Speeds", ChassisSpeeds.struct, state.Speeds);
        SignalLogger.writeDouble("DriveState/OdometryPeriod", state.OdometryPeriod, "seconds");

		BaseStatusSignal.refreshAll(driveCurrentSignals);
		BaseStatusSignal.refreshAll(steerCurrentSignals);

		double totalCurrent = 0;
        /* Telemeterize each module state to a Mechanism2d */
        for (int i = 0; i < 4; ++i) {
			totalCurrent += driveCurrentSignals[i].getValueAsDouble();
			totalCurrent += steerCurrentSignals[i].getValueAsDouble();
        }
		driveTotalCurrent.set(totalCurrent);
		RobotState.driveCurrent = totalCurrent;
		//hub time remaining and active status
		hubTimeRemaining.set(HubTracker.timeRemainingInCurrentShift().map(t -> t.in(edu.wpi.first.units.Units.Seconds)).orElse(0.0));
		hubActive.set(HubTracker.isActive());
		matchTime.set(DriverStation.getMatchTime());
    }
}

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.Drivetrain.ShootOnTheFly;
import frc.robot.subsystems.Shooter.ShooterConstants;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.NetworkTableInstance;

public class RobotState {
	public static Pose2d pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
	public static ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);
	public static Translation2d target = new Translation2d(0.0, 0.0);

	public enum Mode {
			normal,
			manual,			
			climb
		}

	public static Mode mode; 

	public enum Zone {
		ALLIANCE, 
		NEUTRAL,
		LEFT,
		RIGHT,
		TRENCH
	}

	public static boolean isShooting = false;

	private static ShootOnTheFly shootOnTheFly = new ShootOnTheFly();
	public static double lookAheadDistance = 0;
	public static double turretAngle = 0;

	private static double fieldx = 16.54;
	private static double fieldy = 8.07;
	private static double midfieldy = 4.03;

	private static double bluezonex = 4.33;
	private static double redzonex = 12.21;

	public static double driveCurrent = 0;
	public static double climberCurrent = 0;
	public static double intakeCurrent = 0;
	public static double totalRobotCurrent = 0;
	public static double indexerCurrent = 0;
	public static double shooterCurrent = 0;

	private static Translation2d topBlueTrench = new Translation2d(4.614, 7.472);
	private static Translation2d topRedTrench = new Translation2d(11.926, 7.472);
	private static Translation2d bottomBlueTrench = new Translation2d(4.614, 0.637);
	private static Translation2d bottomRedTrench = new Translation2d(11.926, 0.637);

	private static Translation2d[] trenches = new Translation2d[] {topBlueTrench, topRedTrench, bottomBlueTrench, bottomRedTrench};
	private static double trenchdodgeradius = 1;

	public static Zone shooterZone = Zone.ALLIANCE;
	public static Alliance color = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;

	private static final DoublePublisher totalCurrentPub = 
	NetworkTableInstance.getDefault().getTable("HOT/Robot").getDoubleTopic("TotalCurrent").publish();

	public RobotState() {
		mode = Mode.normal; 
	}

	private static boolean isInZoneBox(double xmin, double xmax, double ymin, double ymax) {
		return pose.getX() > xmin && pose.getX() < xmax && pose.getY() > ymin && pose.getY() < ymax;
	}

	private static boolean isInZoneRadius(double x, double y, double radius) {
		return pose.getTranslation().getDistance(new Translation2d(x, y)) < radius;
	}

	public static void update() {
		color = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;
		findZone();
		if (shooterZone == Zone.ALLIANCE || shooterZone == Zone.TRENCH) {
			if (color == Alliance.Red) {
				target = ShooterConstants.redHub;
			} else {
				target = ShooterConstants.blueHub;
			}
		} else {
			if (color == Alliance.Red) {
				if (shooterZone == Zone.RIGHT) {
					target = new Translation2d(15.5, 6.0);// right
				} else {
					target = new Translation2d(15.5, 2.0);// left
				}
			} else {
				if (shooterZone == Zone.RIGHT) {
					target = new Translation2d(1.0, 2.0);// right
				} else {
					target = new Translation2d(1.0, 6.0);// left
				}
			}
		}
	
		shootOnTheFly.update(pose, speeds, target, color);
		turretAngle = shootOnTheFly.getTurretAngle();
		lookAheadDistance = shootOnTheFly.getLookAheadDistance();

		// totalRobotCurrent = driveCurrent + climberCurrent + intakeCurrent + indexerCurrent + shooterCurrent;
		// totalCurrentPub.set(totalRobotCurrent);
	}

	private static void findZone() {
		shooterZone = Zone.NEUTRAL;

		if (color == Alliance.Blue) {
			if (isInZoneBox(0, bluezonex, 0, fieldy)) shooterZone = Zone.ALLIANCE;
			else if (isInZoneBox(bluezonex, fieldx, 0, midfieldy)) shooterZone = Zone.RIGHT;
			else if (isInZoneBox(bluezonex, fieldx, midfieldy, fieldy)) shooterZone = Zone.LEFT;
		} else {
			if (isInZoneBox(redzonex, fieldx, 0, fieldy)) shooterZone = Zone.ALLIANCE;
			else if (isInZoneBox(0, redzonex, 0, midfieldy)) shooterZone = Zone.LEFT;
			else if (isInZoneBox(0, redzonex, midfieldy, fieldy)) shooterZone = Zone.RIGHT;
		}

		double totalSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) / 5.0;

		// if (isInZoneRadius(trenches[0].getX(), trenches[0].getY(), trenchdodgeradius + totalSpeed)) shooterZone = Zone.TRENCH;
		// else if (isInZoneRadius(trenches[1].getX(), trenches[1].getY(), trenchdodgeradius + totalSpeed)) shooterZone = Zone.TRENCH;
		// else if (isInZoneRadius(trenches[2].getX(), trenches[2].getY(), trenchdodgeradius + totalSpeed)) shooterZone = Zone.TRENCH;
		// else if (isInZoneRadius(trenches[3].getX(), trenches[3].getY(), trenchdodgeradius + totalSpeed)) shooterZone = Zone.TRENCH;
	}

	public static Zone getZone() {
		return shooterZone;
	}

	public static void setClimbMode(){
		mode = Mode.climb; 
	}

	public static void setNormalMode (){
		mode = Mode.normal; 
	}

	public static void setManualMode() {
		mode = Mode.manual;
	}

	public static boolean isClimbMode() {
    	return mode == Mode.climb;
	}

	public static boolean isNormalMode() {
    	return mode == Mode.normal;
	}

	public static boolean isManualMode() {
		return mode == Mode.manual;
	}
}


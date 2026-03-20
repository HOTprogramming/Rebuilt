package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.drivetrain.ShootOnTheFly;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.shooter.ShooterConstants;

public class RobotState {

	public static Pose2d pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));
	public static ChassisSpeeds speeds = new ChassisSpeeds(0, 0, 0);

	public enum Mode {
			normal,
			manual,			
			climb
		}

	public	static Mode mode; 


	private enum Zone {
		ALLIANCE, 
		NEUTRAL,
		LEFT,
		RIGHT,
		TRENCH
	}

	private static ShootOnTheFly shootOnTheFly = new ShootOnTheFly();
	public static double lookAheadDistance = 0;
	public static double turretAngle = 0;

	private static double fieldx = 16.54;
	private static double fieldy = 8.07;
	private static double midfieldy = 4.03;

	private static double bluezonex = 4.33;
	private static double redzonex = 12.21;

	private static Translation2d topBlueTrench = new Translation2d(4.614, 7.472);
	private static Translation2d topRedTrench = new Translation2d(11.926, 7.472);
	private static Translation2d bottomBlueTrench = new Translation2d(4.614, 0.637);
	private static Translation2d bottomRedTrench = new Translation2d(11.926, 0.637);

	private static Translation2d[] trenches = new Translation2d[] {topBlueTrench, topRedTrench, bottomBlueTrench, bottomRedTrench};
	private static double trenchdodgeradius = 1;

	public Shooter shooter;

	public static Zone shooterZone = Zone.ALLIANCE;
	private static Map<Zone, Command> neutralShooterCommands;
	private static Supplier<Zone> shooterZoneSup = () -> shooterZone;
	public static Alliance color = DriverStation.getAlliance().isPresent() ? DriverStation.getAlliance().get() : Alliance.Red;

	public RobotState() {

		mode = Mode.normal; 

		shooter = new Shooter();
		neutralShooterCommands = Map.of(
			Zone.ALLIANCE, shooter.maintainFlywheel(), 
			Zone.LEFT, shooter.idle(),
			Zone.RIGHT, shooter.idle(),
			Zone.TRENCH, shooter.dodgeTrench(), 
			Zone.NEUTRAL, shooter.idle());
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
		Translation2d target = new Translation2d(0.0, 0.0);
		if (shooterZone == Zone.ALLIANCE) {
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

		
		SmartDashboard.putNumber("Distance to Goal", target.getDistance(pose.getTranslation()));
		SmartDashboard.putNumberArray("TargetLocation", new Double[] {target.getX(), target.getY()});
		SmartDashboard.putString("Robot Mode", mode.name());
		shootOnTheFly.update(pose, speeds, target, color);
		turretAngle = shootOnTheFly.getTurretAngle();
		lookAheadDistance = shootOnTheFly.getLookAheadDistance();
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

		double totalSpeed = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond)/5.0;

		if (isInZoneRadius(trenches[0].getX(), trenches[0].getY(), trenchdodgeradius + totalSpeed)) shooterZone = Zone.TRENCH;
		else if (isInZoneRadius(trenches[1].getX(), trenches[1].getY(), trenchdodgeradius + totalSpeed)) shooterZone = Zone.TRENCH;
		else if (isInZoneRadius(trenches[2].getX(), trenches[2].getY(), trenchdodgeradius + totalSpeed)) shooterZone = Zone.TRENCH;
		else if (isInZoneRadius(trenches[3].getX(), trenches[3].getY(), trenchdodgeradius + totalSpeed)) shooterZone = Zone.TRENCH;

		SmartDashboard.putString("zone", shooterZone.name());		
	}

	public Command idleCommands() {
		return Commands.select(neutralShooterCommands, shooterZoneSup);
	}

	public void setClimbMode(){
		mode = Mode.climb; 
	}

	public void setNormalMode (){
		mode = Mode.normal; 
	}

	public void setManualMode() {
		mode = Mode.manual;
	}

	public boolean isClimbMode() {
    	return mode == Mode.climb;
	}

	public boolean isNormalMode() {
    	return mode == Mode.normal;
	}

	public boolean isManualMode() {
		return mode == Mode.manual;
	}
}


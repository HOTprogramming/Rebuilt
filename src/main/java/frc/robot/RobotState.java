package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter.Shooter;

public class RobotState {

	public static Pose2d pose = new Pose2d(0, 0, Rotation2d.fromDegrees(0));

	private enum Zone {
		ALLIANCE, 
		NEUTRAL, 
		OPPONENT,
		LEFT,
		RIGHT,
		TRENCH
	}

	private static double fieldx = 16.54;
	private static double fieldy = 8.07;
	private static double midfieldy = 4.03;

	private static double bluezonex = 4.03;
	private static double redzonex = 12.51;

	private static Translation2d topBlueTrench = new Translation2d(4.614, 7.472);
	private static Translation2d topRedTrench = new Translation2d(11.926, 7.472);
	private static Translation2d bottomBlueTrench = new Translation2d(4.614, 0.637);
	private static Translation2d bottomRedTrench = new Translation2d(11.926, 0.637);

	private static Translation2d[] trenches = new Translation2d[] {topBlueTrench, topRedTrench, bottomBlueTrench, bottomRedTrench};
	private static double trenchdodgeradius = 1;

	public Shooter shooter;

	public static Zone shooterZone = Zone.ALLIANCE;
	private static Map<Zone, Command> autoAimCommands;
	private static Supplier<Zone> shooterZoneSup = () -> shooterZone;

	public RobotState() {
		shooter = new Shooter();
		autoAimCommands = Map.of(
			Zone.ALLIANCE, shooter.autoAimHub(), 
			Zone.LEFT, shooter.autoAimLeftCorner(),
			Zone.RIGHT, shooter.autoAimRightCorner(),
			Zone.TRENCH, shooter.dodgeTrench());
	}

	private static boolean isInZoneBox(double xmin, double xmax, double ymin, double ymax) {
		return pose.getX() > xmin && pose.getX() < xmax && pose.getY() > ymin && pose.getY() < ymax;
	}

	private static boolean isInZoneRadius(double x, double y, double radius) {
		return pose.getTranslation().getDistance(new Translation2d(x, y)) < radius;
	}

	static double goalX;

	public static void findZone() {
		Alliance color = DriverStation.getAlliance().get();

		if (color == Alliance.Blue) {
			if (isInZoneBox(0, bluezonex, 0, fieldy)) shooterZone = Zone.ALLIANCE;
			else if (isInZoneBox(bluezonex, fieldx, 0, midfieldy)) shooterZone = Zone.RIGHT;
			else if (isInZoneBox(bluezonex, fieldx, midfieldy, fieldy)) shooterZone = Zone.LEFT;
		} else {
			if (isInZoneBox(redzonex, fieldx, 0, fieldy)) shooterZone = Zone.ALLIANCE;
			else if (isInZoneBox(0, redzonex, 0, midfieldy)) shooterZone = Zone.RIGHT;
			else if (isInZoneBox(0, redzonex, midfieldy, fieldy)) shooterZone = Zone.LEFT;
		}

		if (isInZoneRadius(trenches[0].getX(), trenches[0].getY(), trenchdodgeradius)) shooterZone = Zone.TRENCH;
		else if (isInZoneRadius(trenches[1].getX(), trenches[1].getY(), trenchdodgeradius)) shooterZone = Zone.TRENCH;
		else if (isInZoneRadius(trenches[2].getX(), trenches[2].getY(), trenchdodgeradius)) shooterZone = Zone.TRENCH;
		else if (isInZoneRadius(trenches[3].getX(), trenches[3].getY(), trenchdodgeradius)) shooterZone = Zone.TRENCH;


		SmartDashboard.putString("zone", shooterZone.name());

        if (color == Alliance.Red) {
            goalX = 11.91; //Red hub x
        } else {
            goalX = 4.63; //Blue hub x
        }

        Translation2d goalLocation = new Translation2d(goalX, 4.03); //4.03 is both hubs' y pos

		SmartDashboard.putNumber("Distance to Goal", goalLocation.getDistance(pose.getTranslation()));

	}

	public Command autoAim() {
		return Commands.select(autoAimCommands, shooterZoneSup);
	}
}


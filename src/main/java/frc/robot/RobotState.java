package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class RobotState {

	public static Pose2d pose;

	private enum Zone {
		ALLIANCE, 
		NEUTRAL, 
		OPPONENT
	}

	private static double fieldx = 16.54;
	private static double fieldy = 8.07;
	private static double bluezonex = 4.03;
	private static double redzonex = 12.51;

	private static Translation2d TrenchTopBlue = new Translation2d();

	public static Zone zone;

	private static boolean isInZoneBox(double xmin, double xmax, double ymin, double ymax) {
		return pose.getX() > xmin && pose.getX() < xmax && pose.getY() > ymin && pose.getY() < ymax;
	}

	private static boolean isInZoneRadius(double x, double y, double radius) {
		return pose.getTranslation().getDistance(new Translation2d(x, y)) < radius;
	}

	public static void findZone() {
		if (DriverStation.getAlliance().get() == Alliance.Blue) {
			if (isInZoneBox(0, bluezonex, 0, fieldy)) zone = Zone.ALLIANCE;
			else if (isInZoneBox(bluezonex, redzonex, 0, fieldy)) zone = Zone.NEUTRAL;
			else if (isInZoneBox(redzonex, fieldx, 0, fieldy)) zone = Zone.OPPONENT;
		} else {
			if (isInZoneBox(redzonex, fieldx, 0, fieldy)) zone = Zone.ALLIANCE;
			else if (isInZoneBox(bluezonex, redzonex, 0, fieldy)) zone = Zone.NEUTRAL;
			else if (isInZoneBox(0, bluezonex, 0, fieldy)) zone = Zone.OPPONENT;
		}
	}
}


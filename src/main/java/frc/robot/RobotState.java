package frc.robot;

import java.util.Map;
import java.util.function.Supplier;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Shooter.Shooter;

public class RobotState {

	public static Pose2d pose;

	private enum Zone {
		ALLIANCE, 
		NEUTRAL, 
		OPPONENT,
		LEFT,
		RIGHT
	}

	private static double fieldx = 16.54;
	private static double fieldy = 8.07;
	private static double midfieldy = 4.03;

	private static double bluezonex = 4.03;
	private static double redzonex = 12.51;

	private static Translation2d TrenchTopBlue = new Translation2d();

	public Shooter shooter;

	public static Zone shooterZone;
	private static Map<Zone, Command> autoAimCommands;
	private static Supplier<Zone> shooterZoneSup = () -> shooterZone;

	public RobotState() {
		shooter = new Shooter();
		autoAimCommands.put(Zone.ALLIANCE, shooter.autoAimHub());
		autoAimCommands.put(Zone.LEFT, shooter.autoAimLeftCorner());
		autoAimCommands.put(Zone.RIGHT, shooter.autoAimRightCorner());
	}

	private static boolean isInZoneBox(double xmin, double xmax, double ymin, double ymax) {
		return pose.getX() > xmin && pose.getX() < xmax && pose.getY() > ymin && pose.getY() < ymax;
	}

	private static boolean isInZoneRadius(double x, double y, double radius) {
		return pose.getTranslation().getDistance(new Translation2d(x, y)) < radius;
	}

	public static void findZone() {
		if (DriverStation.getAlliance().get() == Alliance.Blue) {
			if (isInZoneBox(0, bluezonex, 0, fieldy)) shooterZone = Zone.ALLIANCE;
			else if (isInZoneBox(bluezonex, fieldx, 0, midfieldy)) shooterZone = Zone.RIGHT;
			else if (isInZoneBox(bluezonex, fieldx, midfieldy, fieldy)) shooterZone = Zone.LEFT;
		} else {
			if (isInZoneBox(redzonex, fieldx, 0, fieldy)) shooterZone = Zone.ALLIANCE;
			else if (isInZoneBox(0, redzonex, 0, midfieldy)) shooterZone = Zone.RIGHT;
			else if (isInZoneBox(0, redzonex, midfieldy, fieldy)) shooterZone = Zone.LEFT;
		}
	}

	public Command autoAim() {
		return Commands.select(autoAimCommands, shooterZoneSup);
	}
}


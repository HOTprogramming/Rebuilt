package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ShootOnTheFly {
    private static double TOF_CONVERGENCE_THRESHOLD;
    private static double TURRET_ROTATION_OFFSET;

    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
	// filters for vx and vy speeds to smooth sotf target movement
    LinearFilter vxfilt = LinearFilter.movingAverage(4);
    LinearFilter vyfilt = LinearFilter.movingAverage(4);

    private double turretAngle = 0;
    private double lookaheadToTargetDistance = 0;

    static {
        TOF_CONVERGENCE_THRESHOLD = 0.025;
		TURRET_ROTATION_OFFSET = Math.PI; // v3 robot shoots backwards
		//empirically derived time of flight based on distance from target
        timeOfFlightMap.put(1.0, 1.0);
        timeOfFlightMap.put(2.0, 1.1);
        timeOfFlightMap.put(3.0, 1.1);
        timeOfFlightMap.put(4.0, 1.15);
        timeOfFlightMap.put(5.0, 1.15);

        timeOfFlightMap.put(5.1, 1.16);
        timeOfFlightMap.put(17.0, 2.0);
    }

    public void update(Pose2d RobotPos, ChassisSpeeds RobotSpeed, Translation2d goalTranslate2d, Alliance alliance) {
        double latency = 0.1;
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(RobotSpeed, RobotPos.getRotation());
		// filters for vx and vy speeds to smooth sotf target movement
        double vx_filt = vxfilt.calculate(fieldSpeeds.vxMetersPerSecond);
        double vy_filt = vyfilt.calculate(fieldSpeeds.vyMetersPerSecond);
		//pose where the shot is released from the robot, speed times shot latency
        Translation2d releasePos = RobotPos.getTranslation().plus(
                new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond).times(latency));
		//target vector, target minus the shot release position, also determines preliminary shot distance
        Translation2d targetVec = goalTranslate2d.minus(releasePos);
        double dist = targetVec.getNorm();

        int i;
        double timeOfFlight = 0;
        double lastTimeOfFlight = 0;
        Translation2d lookaheadPos = releasePos;
        lookaheadToTargetDistance = dist;
		//iterative loop that calculates actual shot distance based on virtual target
		//checks for convergence of time of flight estimate
        for (i = 0; i < 10; i++) {
			//time of flight based on distance using lookup table
            timeOfFlight = timeOfFlightMap.get(lookaheadToTargetDistance);
			//checks to see if ToF estimate has converged, means target is good
            if (Math.abs(timeOfFlight - lastTimeOfFlight) < TOF_CONVERGENCE_THRESHOLD)
                break;
            lastTimeOfFlight = timeOfFlight;
			//creates an X and Y offset based on current filtered speed and estimated ToF
            double offsetX = vx_filt * timeOfFlight;
            double offsetY = vy_filt * timeOfFlight;
			//adds X and Y offsets to the shot release position
			//this is the point where the robot is virtually shooting from if it were stationary when considering the offsets
            lookaheadPos = releasePos.plus(new Translation2d(offsetX, offsetY));
			//gets shot distance from the virtual shooting point to the target
			//this updated distance gets fed back into the beginning of the loop to estimate new ToF
            lookaheadToTargetDistance = goalTranslate2d.getDistance(lookaheadPos);
        }

        // Aim from the VIRTUAL shooting position to the ACTUAL target
		//This appears to be incorrect? It is just feeding the current robot position and is not using the virtual shooting location?
		//TODO reset this back to the virtual target!
		Rotation2d turretRotation = goalTranslate2d.minus(RobotPos.getTranslation()).getAngle();
		SmartDashboard.putNumber("distance to goal", goalTranslate2d.getDistance(RobotPos.getTranslation()));

        if (Alliance.Red == alliance) {
            turretAngle = turretRotation.getRadians() + Math.PI + TURRET_ROTATION_OFFSET; // v3 robot shoots backwards
        } else {
            turretAngle = turretRotation.getRadians() + TURRET_ROTATION_OFFSET;
        }

        SmartDashboard.putNumber("lookAheadDistance	", lookaheadToTargetDistance);
    }

    public double getTurretAngle() {
        return turretAngle;
    }

    public double getLookAheadDistance() {
        return lookaheadToTargetDistance;
    }
}

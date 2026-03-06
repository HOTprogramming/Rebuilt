package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShootOnTheFly {
    private static double minDistance;
    private static double maxDistance;
    private static double TOF_CONVERGENCE_THRESHOLD;
    private static int MAX_ITERATIONS;
        
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();

    LinearFilter vxfilt = LinearFilter.movingAverage(8);
    LinearFilter vyfilt = LinearFilter.movingAverage(8);

	private double turretAngle = 0;
	private double lookaheadToTargetDistance = 0;

    static {

    minDistance = 1.34;
    maxDistance = 5.60;
    TOF_CONVERGENCE_THRESHOLD = 0.025;
    MAX_ITERATIONS = 10;

    timeOfFlightMap.put(7.0, 1.5);
    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
    timeOfFlightMap.put(1.0, 0.70);    
    }

    
    
    public void update(Pose2d RobotPos, ChassisSpeeds RobotSpeed, boolean allyRed) {
        double latency = 0.1; //needs to be tuned, in seconds
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(RobotSpeed, RobotPos.getRotation());
        double vx_filt = vxfilt.calculate(fieldSpeeds.vxMetersPerSecond) ;
        double vy_filt = vyfilt.calculate(fieldSpeeds.vyMetersPerSecond) ;

        double goalX;
        if (allyRed) {
            goalX = 11.91; //Red hub x
        } else {
            goalX = 4.63; //Blue hub x
        }

        Translation2d goalLocation = new Translation2d(goalX, 4.03); //4.03 is both hubs' y pos

        //position-based shooting solution with iterative TOF solver
        //Use chassis speeds, distance and future pos estimate from above solution
        // Account for imparted velocity by robot (turret) to offset
        int i;
        double timeOfFlight = 0;
        double lastTimeOfFlight = 0;
        Translation2d lookaheadPos = RobotPos.getTranslation().plus(
            new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond).times(latency)
        );
        lookaheadToTargetDistance = goalLocation.minus(lookaheadPos).getNorm();
        
        for (i = 0; i < 10; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadToTargetDistance);

            // Exit if ToF has converged
            if (Math.abs(timeOfFlight - lastTimeOfFlight) < TOF_CONVERGENCE_THRESHOLD) break;
            lastTimeOfFlight = timeOfFlight;
            
            // Predict future robot position at time of impact
            double offsetX =  vx_filt * timeOfFlight;
            double offsetY = vy_filt * timeOfFlight;
            lookaheadPos = lookaheadPos.plus(new Translation2d(offsetX, offsetY));

            // Update distance for next ToF lookup
            lookaheadToTargetDistance = goalLocation.getDistance(lookaheadPos);
        }

        // Calculate "Virtual Target" (Iterative Aim Point) for visualization
        // It's the Goal minus the Robot's displacement during flight
        Translation2d robotDisplacement = lookaheadPos.minus(lookaheadPos);
        Translation2d aimPointIterative = goalLocation.minus(robotDisplacement);
        
        //Aim from the CURRENT position to the VIRTUAL target
        Rotation2d turretRotation = goalLocation.minus(lookaheadPos).getAngle();
        turretAngle = turretRotation.getRadians() + Math.PI; // Backwards compensation
    }

	public double getTurretAngle() {
		return turretAngle;
	}

	public double getLookAheadDistance() {
		return lookaheadToTargetDistance;
	}
}

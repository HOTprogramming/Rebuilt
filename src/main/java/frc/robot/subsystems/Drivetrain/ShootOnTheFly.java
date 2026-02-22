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
    private static final InterpolatingTreeMap<Double, Rotation2d> shotHoodAngleMap =
        new InterpolatingTreeMap<>(InverseInterpolator.forDouble(), Rotation2d::interpolate);
    private static final InterpolatingDoubleTreeMap shotFlywheelSpeedMap =
        new InterpolatingDoubleTreeMap();
    LinearFilter vxfilt = LinearFilter.movingAverage(8);
    LinearFilter vyfilt = LinearFilter.movingAverage(8);

	static {
        // Updated based on sim characterization results
        minDistance = 1.34;
        maxDistance = 5.57;
        TOF_CONVERGENCE_THRESHOLD = 0.025;
        MAX_ITERATIONS = 10;

        // Hood Angles (Note: Conversion to Rotation2d from Degrees)
        shotHoodAngleMap.put(1.34, Rotation2d.fromDegrees(79.00));
        shotHoodAngleMap.put(2.17, Rotation2d.fromDegrees(63.00));
        shotHoodAngleMap.put(3.15, Rotation2d.fromDegrees(63.00));
        shotHoodAngleMap.put(4.09, Rotation2d.fromDegrees(60.00));
        shotHoodAngleMap.put(5.57, Rotation2d.fromDegrees(53.00));

        // Flywheel Speeds (MPS)
        shotFlywheelSpeedMap.put(1.34, 6.25);
        shotFlywheelSpeedMap.put(2.17, 6.50);
        shotFlywheelSpeedMap.put(3.15, 7.00);
        shotFlywheelSpeedMap.put(4.09, 7.60);
        shotFlywheelSpeedMap.put(5.57, 8.50);

        // Time of Flight (Seconds - Measured)
        timeOfFlightMap.put(1.34, 0.92);
        timeOfFlightMap.put(2.17, 0.92);
        timeOfFlightMap.put(3.15, 0.96);
        timeOfFlightMap.put(4.09, 1.06);
        timeOfFlightMap.put(5.57, 1.12);
    }

    public static double getFlywheelSpeed(double distance) {
		return shotFlywheelSpeedMap.get(distance);
	}
	
	public static Rotation2d getHoodAngle(double distance) {
		return shotHoodAngleMap.get(distance);
	}
    
    public double update(Pose2d RobotPos, ChassisSpeeds RobotSpeed, boolean allyRed) {
        double latency = 0.2; //needs to be tuned, in seconds
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(RobotSpeed, RobotPos.getRotation());
        double vx_filt = vxfilt.calculate(fieldSpeeds.vxMetersPerSecond) ;
        double vy_filt = vyfilt.calculate(fieldSpeeds.vyMetersPerSecond) ;

        Translation2d releasePos = RobotPos.getTranslation().plus(
            new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond).times(latency)
        );


        double goalX;
        if (allyRed) {
            goalX = 11.91; //Red hub x
        } else {
            goalX = 4.63; //Blue hub x
        }

        Translation2d goalLocation = new Translation2d(goalX, 4.03); //4.03 is both hubs' y pos
        Translation2d targetVec = goalLocation.minus(releasePos);
        double dist = targetVec.getNorm();
        Translation2d targetDirection = targetVec.div(dist); 
        
        //double idealHorizontalSpeed = 5; old code
        double initialToF = timeOfFlightMap.get(dist);
        double idealHorizontalSpeed = dist / initialToF;

        Translation2d targetVelocity = targetDirection.times(idealHorizontalSpeed);
        
        Translation2d robotVelVec = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        Translation2d shotVec = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);
        Translation2d shotVelocity = targetVelocity.minus(robotVelVec);

        //Rotation2d turretRotation = shotVelocity.getAngle();
        double requiredVelocity = shotVelocity.getNorm();
        //double turretAngle = turretRotation.getRadians() + Math.PI; // Backwards compensation

        Translation2d aimPoint = releasePos.plus(
            shotVelocity.div(shotVelocity.getNorm()).times(dist)
        );

        /* Old code for non-iterative solution, aiming at the current position of the goal
        Translation2d aimPoint = releasePos.plus(
            shotVec.div(shotVec.getNorm()).times(dist)
        );
        */

        double newHorizontalSpeed = shotVelocity.getNorm();

        double totalExitVelocity = 15; //In m/s, needs to be measured when we get robot
        double ratio = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
        double newPitch = Math.acos(ratio);
        

        //position-based shooting solution with iterative TOF solver
        //Use chassis speeds, distance and future pos estimate from above solution
        // Account for imparted velocity by robot (turret) to offset
        int i;
        double timeOfFlight = 0;
        double lastTimeOfFlight = 0;
        Translation2d lookaheadPos = releasePos;
        double lookaheadToTargetDistance = dist;
        
        for (i = 0; i < 10; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadToTargetDistance);

            // Exit if ToF has converged
            if (Math.abs(timeOfFlight - lastTimeOfFlight) < TOF_CONVERGENCE_THRESHOLD) break;
            lastTimeOfFlight = timeOfFlight;
            
            // Predict future robot position at time of impact
            double offsetX =  vx_filt * timeOfFlight;
            double offsetY = vy_filt * timeOfFlight;
            lookaheadPos = releasePos.plus(new Translation2d(offsetX, offsetY));

            // Update distance for next ToF lookup
            lookaheadToTargetDistance = goalLocation.getDistance(lookaheadPos);
        }

        // Calculate "Virtual Target" (Iterative Aim Point) for visualization
        // It's the Goal minus the Robot's displacement during flight
        Translation2d robotDisplacement = lookaheadPos.minus(releasePos);
        Translation2d aimPointIterative = goalLocation.minus(robotDisplacement);
        
        //Aim from the CURRENT position to the VIRTUAL target
        Rotation2d turretRotation = goalLocation.minus(lookaheadPos).getAngle();
        double turretAngle = turretRotation.getRadians() + Math.PI; // Backwards compensation
        
        SmartDashboard.putNumberArray("Aim Point (vel)", new double[] {aimPoint.getX(), aimPoint.getY()}); 
        SmartDashboard.putNumberArray("Aim Point (iterative)", new double[] {aimPointIterative.getX(), aimPointIterative.getY()}); 
        SmartDashboard.putNumber("Shooting Angle", Math.toDegrees(turretAngle));
        SmartDashboard.putNumber("SolverErr", Math.abs(timeOfFlight - lastTimeOfFlight));
        SmartDashboard.putNumber("SolverLoops", i);
        SmartDashboard.putNumber("Distance", dist);
        SmartDashboard.putNumber("TimeOfFlight", timeOfFlight);
        SmartDashboard.putNumberArray("LaunchPos", new double [] {lookaheadPos.getX(), lookaheadPos.getY()});
        SmartDashboard.putNumber("vx", fieldSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("vy", fieldSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("vx_filtered", vx_filt);
        SmartDashboard.putNumber("vy_filtered", vy_filt);
        SmartDashboard.putNumber("Horiz Speed", newHorizontalSpeed);

        return(turretAngle);

    }
}

package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShootingWhileMoving {
    private static final InterpolatingDoubleTreeMap timeOfFlightMap = new InterpolatingDoubleTreeMap();
    
    static {
    timeOfFlightMap.put(5.68, 1.16);
    timeOfFlightMap.put(4.55, 1.12);
    timeOfFlightMap.put(3.15, 1.11);
    timeOfFlightMap.put(1.88, 1.09);
    timeOfFlightMap.put(1.38, 0.90);
    }


    public double update(Pose2d RobotPos, ChassisSpeeds RobotSpeed, boolean allyRed) {
        double latency = 0.1; //needs to be tuned, in seconds
        ChassisSpeeds fieldSpeeds = ChassisSpeeds.fromRobotRelativeSpeeds(RobotSpeed, RobotPos.getRotation()); 
        Translation2d futurePos = RobotPos.getTranslation().plus(
            new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond).times(latency)
        );


        double goalX;
        if (allyRed) {
            goalX = 11.91; //Red hub x
        } else {
            goalX = 4.63; //Blue hub x
        }

        Translation2d goalLocation = new Translation2d(goalX, 4.03); //4.03 is both hubs' y pos
        Translation2d targetVec = goalLocation.minus(futurePos);
        double dist = targetVec.getNorm();

        double idealHorizontalSpeed = 5;

        Translation2d robotVelVec = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        Translation2d shotVec = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

        Translation2d aimPoint = futurePos.plus(
            shotVec.div(shotVec.getNorm()).times(dist)
        );

        

        double turretAngle = shotVec.getAngle().getRadians() + Math.PI; //Turret faces backwards on robot
        double newHorizontalSpeed = shotVec.getNorm();

        

        double totalExitVelocity = 15; //In m/s, needs to be measured when we get robot
        double ratio = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
        double newPitch = Math.acos(ratio);
        
        

        //position-based shooting solution with iterative TOF solver
        //Use chassis speeds, distance and future pos estimate from above solution
        // Account for imparted velocity by robot (turret) to offset
        double timeOfFlight;
        Translation2d lookaheadPos = futurePos;
        double lookaheadToTargetDistance = dist;
        for (int i = 0; i < 5; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadToTargetDistance);
            double offsetX =  fieldSpeeds.vxMetersPerSecond * timeOfFlight;
            double offsetY = fieldSpeeds.vyMetersPerSecond * timeOfFlight;
            lookaheadPos = futurePos.plus(new Translation2d(offsetX, offsetY));
            lookaheadToTargetDistance = goalLocation.getDistance(lookaheadPos);
        }

        //Calculate final shot parameters
        //double turretAngle = goalLocation.minus(lookaheadPos).getAngle().getRadians() + Math.PI; //Turret faces backwards on robot
        
        SmartDashboard.putNumberArray("Aim Point (vel)", new double[] {aimPoint.getX(), aimPoint.getY()}); 
        SmartDashboard.putNumberArray("Aim Point (iterative)", new double[] {lookaheadPos.getX(), lookaheadPos.getY()}); 
        SmartDashboard.putNumber("Shooting Angle", Math.toDegrees(turretAngle));

        return(turretAngle);

    }
}

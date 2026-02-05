package frc.robot.subsystems.Drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class ShootingWhileMoving {
    
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

        double idealHorizontalSpeed = 10;

        Translation2d robotVelVec = new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond);
        Translation2d shotVec = targetVec.div(dist).times(idealHorizontalSpeed).minus(robotVelVec);

        Translation2d aimPoint = RobotPos.getTranslation().plus(
            shotVec.div(shotVec.getNorm()).times(dist)
        );

        SmartDashboard.putNumberArray("Target Vec", new double[] {aimPoint.getX(), aimPoint.getY()}); 

        double turretAngle = shotVec.getAngle().getRadians() + Math.PI; //Turret faces backwards on robot
        double newHorizontalSpeed = shotVec.getNorm();

        SmartDashboard.putNumber("Shooting Angle", Math.toDegrees(turretAngle));

        double totalExitVelocity = 15; //In m/s, needs to be measured when we get robot
        double ratio = Math.min(newHorizontalSpeed / totalExitVelocity, 1.0);
        double newPitch = Math.acos(ratio);

        return(turretAngle);
    }
}

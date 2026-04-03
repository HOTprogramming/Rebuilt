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

    LinearFilter vxfilt = LinearFilter.movingAverage(8);
    LinearFilter vyfilt = LinearFilter.movingAverage(8);

    private double turretAngle = 0;
    private double lookaheadToTargetDistance = 0;

    static {
        TOF_CONVERGENCE_THRESHOLD = 0.025;
		TURRET_ROTATION_OFFSET = 0.0; // was Math.PI in v2 robot

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
        double vx_filt = vxfilt.calculate(fieldSpeeds.vxMetersPerSecond);
        double vy_filt = vyfilt.calculate(fieldSpeeds.vyMetersPerSecond);

        Translation2d releasePos = RobotPos.getTranslation().plus(
                new Translation2d(fieldSpeeds.vxMetersPerSecond, fieldSpeeds.vyMetersPerSecond).times(latency));

        Translation2d targetVec = goalTranslate2d.minus(releasePos);
        double dist = targetVec.getNorm();

        int i;
        double timeOfFlight = 0;
        double lastTimeOfFlight = 0;
        Translation2d lookaheadPos = releasePos;
        lookaheadToTargetDistance = dist;

        for (i = 0; i < 10; i++) {
            timeOfFlight = timeOfFlightMap.get(lookaheadToTargetDistance);

            if (Math.abs(timeOfFlight - lastTimeOfFlight) < TOF_CONVERGENCE_THRESHOLD)
                break;
            lastTimeOfFlight = timeOfFlight;

            double offsetX = vx_filt * timeOfFlight;
            double offsetY = vy_filt * timeOfFlight;
            lookaheadPos = releasePos.plus(new Translation2d(offsetX, offsetY));

            lookaheadToTargetDistance = goalTranslate2d.getDistance(lookaheadPos);
        }

        // Aim from the CURRENT position to the VIRTUAL target
        Rotation2d turretRotation = goalTranslate2d.minus(lookaheadPos).getAngle();
        if (Alliance.Red == alliance) {
            turretAngle = turretRotation.getRadians() + TURRET_ROTATION_OFFSET; // v3 robot shoots backwards
        } else {
            turretAngle = turretRotation.getRadians();
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

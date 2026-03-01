package frc.robot.subsystems.Camera;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;

/**
 * Helper utilities to compute measurement standard deviations for vision tag-based pose updates.
 *
 * Usage: call computeScaledStdDevs(...) with the camera pose estimate and the known tag field
 * pose to get a {sigmaX_m, sigmaY_m, sigmaTheta_rad} array to pass into your pose estimator.
 */
public final class TagMeasurementUtils {
    private TagMeasurementUtils() {}

    public static class Tuning {
        public double baseX = 0.05; // meters
        public double baseY = 0.05; // meters
        public double baseTheta = Math.toRadians(3.0); // radians

        public double kDistance = 0.08; // linear multiplier per meter
        public double kDistanceSquared = 0.0; // optional quadratic term
        public double kAngle = 2.0; // angle-based multiplier

        public double minXY = 0.02; // meters
        public double maxXY = 1.5; // meters
        public double minTheta = Math.toRadians(0.5);
        public double maxTheta = Math.toRadians(60.0);
    }

    /**
     * Compute measurement standard deviations (x meters, y meters, theta radians) for a vision
     * pose derived from seeing an AprilTag at tagPoseOnField from cameraPoseOnField.
     *
     * cameraPoseOnField: the robot/camera pose in field coordinates (estimated).
     * tagPoseOnField: the known, fixed tag pose on the field.
     * cameraPoseStd: optional three-element array {sigmaX, sigmaY, sigmaTheta} representing
     *                uncertainty of the camera pose estimate (use null if unknown).
     * tuning: tuning constants (nullable - will use defaults if null).
     *
     * Returns: double[3] = {sigmaX_m, sigmaY_m, sigmaTheta_rad}
     */
    public static double[] computeScaledStdDevs(
            Pose2d cameraPoseOnField,
            Pose2d tagPoseOnField,
            double[] cameraPoseStd, // nullable
            Tuning tuning // nullable
    ) {
        Tuning t = tuning == null ? new Tuning() : tuning;

        // Defensive fallback
        if (cameraPoseOnField == null || tagPoseOnField == null) {
            return new double[] {t.baseX, t.baseY, t.baseTheta};
        }

        Translation2d camT = cameraPoseOnField.getTranslation();
        Translation2d tagT = tagPoseOnField.getTranslation();

        double distance = camT.getDistance(tagT); // meters

        // Distance multiplier (linear + optional quadratic)
        double distanceFactor = 1.0 + t.kDistance * distance + t.kDistanceSquared * distance * distance;

        // Compute incidence angle between camera forward and vector to tag
        Rotation2d camYaw = cameraPoseOnField.getRotation();
        Translation2d toTag = tagT.minus(camT);
        if (Math.abs(toTag.getX()) < 1e-9 && Math.abs(toTag.getY()) < 1e-9) {
            double sx = Math.max(t.minXY, t.baseX);
            double sy = Math.max(t.minXY, t.baseY);
            double st = Math.max(t.minTheta, t.baseTheta);
            return new double[] {sx, sy, st};
        }
        Rotation2d toTagAngle = toTag.getAngle();

        double angleDiff = Math.abs(camYaw.minus(toTagAngle).getRadians()); // 0 = head-on
        // normalize to [0, pi]
        angleDiff = Math.min(angleDiff, Math.abs(2.0 * Math.PI - angleDiff));

        // Angle factor: more oblique => larger uncertainty
        double angleFactor = 1.0 + t.kAngle * (1.0 - Math.cos(angleDiff));
        if (Math.cos(angleDiff) < 0.3) {
            angleFactor *= 1.5; // strongly oblique
        }

        // Compute measurement-only stdevs
        double measX = t.baseX * distanceFactor * angleFactor;
        double measY = t.baseY * distanceFactor * angleFactor;
        double measTheta = t.baseTheta * distanceFactor * angleFactor;

        // Combine with camera pose uncertainty (if provided)
        double camX = 0.0, camY = 0.0, camTheta = 0.0;
        if (cameraPoseStd != null && cameraPoseStd.length >= 3) {
            camX = cameraPoseStd[0];
            camY = cameraPoseStd[1];
            camTheta = cameraPoseStd[2];
        }

        double totalX = Math.sqrt(measX * measX + camX * camX);
        double totalY = Math.sqrt(measY * measY + camY * camY);
        double totalTheta = Math.sqrt(measTheta * measTheta + camTheta * camTheta);

        // Clamp to min/max
        totalX = clamp(totalX, t.minXY, t.maxXY);
        totalY = clamp(totalY, t.minXY, t.maxXY);
        totalTheta = clamp(totalTheta, t.minTheta, t.maxTheta);

        // Protect against NaN / zero
        if (!Double.isFinite(totalX) || totalX <= 0.0) totalX = t.minXY;
        if (!Double.isFinite(totalY) || totalY <= 0.0) totalY = t.minXY;
        if (!Double.isFinite(totalTheta) || totalTheta <= 0.0) totalTheta = t.minTheta;

        return new double[] {totalX, totalY, totalTheta};
    }

    private static double clamp(double v, double lo, double hi) {
        if (Double.isNaN(v)) return lo;
        return Math.max(lo, Math.min(hi, v));
    }
}

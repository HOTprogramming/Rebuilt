
package frc.robot.subsystems.Camera;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.proto.Photon.ProtobufPhotonTrackedTarget;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N4;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StringPublisher;

import static edu.wpi.first.units.Units.Inch;

import java.beans.VetoableChangeListener;
import java.io.IOException;
import java.time.Month;
import java.util.ArrayList;
import java.util.EnumMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

import javax.xml.crypto.Data;

import frc.robot.Robot;
import frc.robot.subsystems.Camera.CameraConstants.CameraConstant;
import frc.robot.subsystems.Drivetrain.Drive;
public class Camera extends SubsystemBase {

    public enum CameraPositions {
        FRONT,
        LEFT,
        RIGHT,
        BACK,
        TOP
    }

    // front
    PhotonCamera frontCamera;
    PhotonPipelineResult result;
    PhotonTrackedTarget frontBestTarget;
    double frontSampleTime;
    Pose2d frontCamEstimation;
    Matrix<N3, N1> frontDevs = VecBuilder.fill(1, 1, 1);

    // left
    PhotonCamera leftCamera;
    PhotonPipelineResult leftResult;
    PhotonTrackedTarget leftBestTarget;
    double leftSampleTime;
    Pose2d leftCamEstimation;
    Matrix<N3, N1> leftDevs = VecBuilder.fill(1, 1, 1);

    // right
    PhotonCamera rightCamera;
    PhotonPipelineResult rightResult;
    PhotonTrackedTarget rightBestTarget;
    double rightSampleTime;
    Pose2d rightCamEstimation;
    Matrix<N3, N1> rightDevs = VecBuilder.fill(1, 1, 1);

    // rear
    PhotonCamera rearCamera;
    PhotonPipelineResult rearResult;
    PhotonTrackedTarget rearBestTarget;
    double rearSampleTime;
    Pose2d rearCamEstimation;
    Matrix<N3, N1> rearDevs = VecBuilder.fill(1, 1, 1);

    
    AprilTagFieldLayout tag;

    boolean drawWireframes = false; // resource heavy

    Nat<N3> rows = new Nat<N3>() {

        @Override
        public int getNum() {
            return 3;
        }
        
    };
    Nat<N4> colls = new Nat<N4>() {

        @Override
        public int getNum() {
            return 4;
        }
        
    };

    Matrix<N3, N4> stDevs = new Matrix<N3, N4>(rows, colls);
    double[] timestamps = new double[] {-1, -1, -1 ,-1};
    Pose2d[] poses = new Pose2d[] {
        new Pose2d(),
        new Pose2d(),
        new Pose2d(),
        new Pose2d()
    };



    
    // NetworkTables
    NetworkTableInstance instance = NetworkTableInstance.getDefault();
    NetworkTable table = instance.getTable("Vision");
    StringPublisher fieldTypePublisher = table.getStringTopic(".type").publish();

    DoubleArrayPublisher frontCameraPub;
    DoubleArrayPublisher leftCameraPub;
    DoubleArrayPublisher rightCameraPub;
    DoubleArrayPublisher rearCameraPub;

    Field2d fieldRight = new Field2d();
    Field2d fieldLeft = new Field2d();

    Map<CameraPositions, Optional<EstimatedRobotPose>> cameraMeasurements = new EnumMap<>(CameraPositions.class);
    Map<CameraPositions, Matrix<N3, N1>> cameraStdDeviations = new EnumMap<>(CameraPositions.class);

    Map<CameraPositions, PhotonCamera> cameras = new EnumMap<>(CameraPositions.class);
    Map<CameraPositions, DoubleArrayPublisher> publishers = new EnumMap<>(CameraPositions.class);
    Map<CameraPositions, PhotonPoseEstimator> photonPoseEstimators = new EnumMap<>(CameraPositions.class); 
    Map<CameraPositions, Double> lastEstTimestamps = new EnumMap<>(CameraPositions.class); 
    private Map<CameraPositions, List<PhotonTrackedTarget>> targetsSeenByCamera  = new EnumMap<>(CameraPositions.class);
    CameraPositions allowMultiTag;

    private int minimumTagsSeenByAnyCamera;
    private int lastMinimumTagsSeenByAnyCamera = 0;

    private int loopsPast;

    private CameraConstants constants;

    private AprilTagFieldLayout tags;

    // docs https://docs.photonvision.org/ 
    private Drive drivetrain;

    private boolean ignoring = false;

    public Camera(Drive drive) {
        SmartDashboard.putData("Left", fieldLeft);
        SmartDashboard.putData("Right", fieldRight);

        drivetrain = drive;
        constants = new CameraConstants();
        
        tags = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        CameraConstant cameraConstant = constants.cameraConstants.get(CameraPositions.FRONT);
        if (cameraConstant != null) {
                cameras.put(CameraPositions.FRONT, new PhotonCamera(cameraConstant.getName()));
                publishers.put(CameraPositions.FRONT, table.getDoubleArrayTopic("Front_Camera").publish());
                PhotonPoseEstimator estimator = new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraConstant.getTransform());
                estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                photonPoseEstimators.put(CameraPositions.FRONT, estimator);
                lastEstTimestamps.put(CameraPositions.FRONT, -1.0);
        }

        cameraConstant = constants.cameraConstants.get(CameraPositions.LEFT);
        if (cameraConstant != null) {
            cameras.put(CameraPositions.LEFT, new PhotonCamera(cameraConstant.getName()));
            publishers.put(CameraPositions.LEFT, table.getDoubleArrayTopic("Left_Camera").publish());
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraConstant.getTransform());
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            photonPoseEstimators.put(CameraPositions.LEFT, estimator);
                lastEstTimestamps.put(CameraPositions.LEFT, -1.0);
        }
        
        cameraConstant = constants.cameraConstants.get(CameraPositions.RIGHT);
        if (cameraConstant != null) {
            cameras.put(CameraPositions.RIGHT, new PhotonCamera(cameraConstant.getName()));
            publishers.put(CameraPositions.RIGHT, table.getDoubleArrayTopic("Right_Camera").publish());
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraConstant.getTransform());
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            photonPoseEstimators.put(CameraPositions.RIGHT, estimator);
                lastEstTimestamps.put(CameraPositions.RIGHT, -1.0);
        }
        
        cameraConstant = constants.cameraConstants.get(CameraPositions.BACK);
        if (cameraConstant != null) {
            cameras.put(CameraPositions.BACK, new PhotonCamera(cameraConstant.getName()));
            publishers.put(CameraPositions.BACK, table.getDoubleArrayTopic("Back_Camera").publish());
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(tags, PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraConstant.getTransform());
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
                lastEstTimestamps.put(CameraPositions.BACK, -1.0);
            photonPoseEstimators.put(CameraPositions.BACK, estimator);
        }
    }

    public Command usetIgnore() {
        return runOnce(() -> ignoring = true);
    }

    public Command suetUnIgnore() {
        return runOnce(() -> ignoring = false);
    }

    private Optional<EstimatedRobotPose> updateCameraMeasurment(CameraPositions key, CameraConstant constant, PhotonCamera camera, DoubleArrayPublisher publisher, PhotonPoseEstimator estimator, double lastEstTimestamp) {
        var visionEst = estimator.update(camera.getLatestResult());
        double latestTimestamp = camera.getLatestResult().getTimestampSeconds();
        boolean newResult = Math.abs(latestTimestamp - lastEstTimestamp) > 1e-5;

        targetsSeenByCamera.put(key,camera.getLatestResult().targets);

        if (newResult) {
            lastEstTimestamp = latestTimestamp;
        } else {
            publisher.set(null);
        }
            visionEst.ifPresent(est -> { 
                publisher.set(new double[] {est.estimatedPose.toPose2d().getX(),
                est.estimatedPose.toPose2d().getY(),
                est.estimatedPose.toPose2d().getRotation().getDegrees()});
            });
            return visionEst;
    }

    public boolean getCameraAlive(CameraPositions camera) {
        return cameras.get(camera).isConnected();
    }

    // public boolean getCameraSeesTag(CameraPositions camera) {
    //     return 
    // }

     @Override
    public void periodic() {

        //drivetrain.seesReefTag = false;

        minimumTagsSeenByAnyCamera = 0;

        constants.cameraConstants.forEach((key,constant) -> {
            if (constant != null) {
                cameraMeasurements.put(key,updateCameraMeasurment(key, constant, cameras.get(key), publishers.get(key), photonPoseEstimators.get(key), lastEstTimestamps.get(key)));
            }
        });

        targetsSeenByCamera.forEach((key, list) -> {
            if (list.size() > minimumTagsSeenByAnyCamera) {
                minimumTagsSeenByAnyCamera = list.size();
            }
            SmartDashboard.putNumber("Camera" + key + "sees:", list.size());
        });

        allowMultiTag = CameraPositions.BACK;

        if (targetsSeenByCamera.containsKey(CameraPositions.BACK) && targetsSeenByCamera.get(CameraPositions.BACK).size() >=2 ) {
            allowMultiTag = CameraPositions.BACK;
        } else if (targetsSeenByCamera.containsKey(CameraPositions.RIGHT) && targetsSeenByCamera.get(CameraPositions.RIGHT).size() >=2 ) {
            allowMultiTag = CameraPositions.RIGHT;
        } else if (targetsSeenByCamera.containsKey(CameraPositions.LEFT) && targetsSeenByCamera.get(CameraPositions.LEFT).size() >=2 ) {
            allowMultiTag = CameraPositions.LEFT;
        } else if (targetsSeenByCamera.containsKey(CameraPositions.FRONT) && targetsSeenByCamera.get(CameraPositions.FRONT).size() >=2 ) {
            allowMultiTag = CameraPositions.FRONT;
        } 

        cameraMeasurements.forEach((key,measurement) -> {
            if (measurement.isPresent()) {
                cameraStdDeviations.put(key,getEstimationStdDevs(measurement.get().estimatedPose.toPose2d(), cameras.get(key), constants.cameraConstants.get(key), photonPoseEstimators.get(key),allowMultiTag == key));
            }
        });

        // robotState.putTargetsSeenByCamera(targetsSeenByCamera);

        cameraMeasurements.forEach((key,measurement) -> {
            // ignore top camera when aligning to reef
                measurement.ifPresent(
                    est -> {
                        // Change our trust in the measurement based on the tags we can see
                        var estStdDevs = cameraStdDeviations.get(key);

                        drivetrain.addVisionMeasurement(
                                est.estimatedPose.toPose2d(), est.timestampSeconds, estStdDevs);
                        
                        // SmartDashboard.putNumber(key + " X", est.estimatedPose.toPose2d().getX());
                        // SmartDashboard.putNumber(key + " Y", est.estimatedPose.toPose2d().getY());
                        // SmartDashboard.putNumber(key + " Rotation", est.estimatedPose.toPose2d().getRotation().getDegrees());

                        if(key == CameraPositions.RIGHT){
                            fieldRight.setRobotPose(est.estimatedPose.toPose2d()); 
                        } else {
                            fieldLeft.setRobotPose(est.estimatedPose.toPose2d()); 
                        }

                    });
            
            });
        // robotState.setVisionMeasurements(cameraMeasurements);
        // robotState.setCameraStdDeviations(cameraStdDeviations);
    }

    public Matrix<N3, N1> getEstimationStdDevs(Pose2d estimatedPose, PhotonCamera camera, CameraConstant constant, PhotonPoseEstimator estimator, boolean allowMultiTag) {
        var estStdDevs = constant.getSingleTagStdDevs();
        var targets = camera.getLatestResult().getTargets();
        int numTags = 0;
        double avgDist = 0;
        for (var tgt : targets) {
            var tagPose = estimator.getFieldTags().getTagPose(tgt.getFiducialId());
            if (tagPose.isEmpty()) continue;
            numTags++;
            avgDist +=
                    tagPose.get().toPose2d().getTranslation().getDistance(estimatedPose.getTranslation());
        }
        if (numTags == 0) return estStdDevs;
        avgDist /= numTags;
        // Decrease std devs if multiple targets are visible
        if (allowMultiTag && numTags > 1) estStdDevs = constant.getMultiTagStdDevs();;
        // Increase std devs based on (average) distance
        if (numTags > 1 && avgDist > 6.5) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        if (numTags == 1 && avgDist > 4)
            estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        else estStdDevs = estStdDevs.times(1 + (avgDist * avgDist / 30));

        if (numTags == 1 && avgDist > 2.75) {
            estStdDevs =VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
        }


        return estStdDevs;
    }    
}

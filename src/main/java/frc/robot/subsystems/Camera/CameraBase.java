package frc.robot.subsystems.Camera;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotState;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.Vector;

public class CameraBase {
	private Drivetrain drivetrain;

	NetworkTable subTable;
	StructTopic<Pose2d> pose2dTopic;
	StructSubscriber<Pose2d> subscriber;
	AprilTagFieldLayout layout = AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
	Vector<N3> estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
	Vector<N3> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
	Vector<N3> twoTagStDevs = VecBuilder.fill(.6, .6, 1);
	Vector<N3> threeTagStDevs = VecBuilder.fill(.55, .55, 1);
	Vector<N3> fourTagStdDevs = VecBuilder.fill(.5, .5, 1);
	Pose2d poseArray;
	double avgDist;
	Number[] tagIds;
	double numTags;
	Translation2d prevTranslation = new Translation2d(0, 0);
	String name;
	Pose2d prevPose = new Pose2d();

	public CameraBase(Drivetrain drive, String name) {
		drivetrain = drive;
		this.name = name;

		NetworkTable hotVisionTable = NetworkTableInstance.getDefault().getTable("HOTVision");

		subTable = hotVisionTable.getSubTable(name + "/apriltag");
		pose2dTopic = subTable.getStructTopic("robot_pose2d", Pose2d.struct);
		subscriber = pose2dTopic.subscribe(new Pose2d());
		SmartDashboard.putBoolean("climb/Use Cam Whl Dock", false);
	}

	public int[] blueTargetTags = {27, 18, 25, 26, 24, 21, 29, 30, 31, 32};
	public int[] redTargetTags = {9, 10, 8, 5, 11, 2, 16, 15, 14, 13};
	public int[] targetTags = {};

	public void periodic() {
		if(subTable.getEntry("has_pose").getBoolean(false)){
			double latency = subTable.getEntry("latency").getNumber(0).floatValue() / 1000.0;
			poseArray = subscriber.get();
			long numTags = subTable.getEntry("num_tags").getInteger(0);
			tagIds = subTable.getEntry("tag_ids").getNumberArray(new Number[0]);
			avgDist = 0;
			Number ambg = subTable.getEntry("ambiguity").getNumber(0);
			boolean targeting = false;

			if(RobotState.color == Alliance.Red){
				targetTags = redTargetTags;
			} else {
				targetTags = blueTargetTags;
			}

			for(Number id : tagIds){
				avgDist += layout.getTagPose(id.intValue()).get().getTranslation().toTranslation2d().
					getDistance(poseArray.getTranslation());
				
				if(RobotState.isShooting){
					for(int targetId : targetTags){
						if(targetId == id.intValue()){
							targeting = true;
						}
					}
				}
			}

			if(numTags > 0){
				avgDist = avgDist / (double) numTags;
			}

			estStdDevs = (numTags > 1) ? twoTagStDevs : singleTagStdDevs;

			if (numTags == 1) {
				if (avgDist > 2.5) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				else estStdDevs = singleTagStdDevs;
			} else if (numTags == 2) {
				if (avgDist > 5.5) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				else estStdDevs = twoTagStDevs;
			} else if (numTags == 3) {
				if (avgDist > 8) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				else if (avgDist > 6.5) estStdDevs = threeTagStDevs.times(1.25);
				else estStdDevs = threeTagStDevs;
			} else if (numTags > 3) {
				if (avgDist > 10) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				else if (avgDist > 7.5) estStdDevs = fourTagStdDevs.times(1.25);
				else estStdDevs = fourTagStdDevs;
			}

			estStdDevs = estStdDevs.times(1 + ((avgDist * avgDist / 30.0)));

			if(ambg.doubleValue() > .2){
				estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
			}

			if(prevPose.getTranslation().getDistance(poseArray.getTranslation()) > 1 && prevPose.getX() != 0){
				estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
			}

			if(targeting && avgDist < 4){
				estStdDevs = estStdDevs.div(3 * (1 + (poseArray.getTranslation().getDistance(drivetrain.getState().Pose.getTranslation()))));
			}

			if(poseArray.getX() < 20 && poseArray.getX() > 0 && poseArray.getY() < 20 && poseArray.getY() > 0 && numTags > 0){
				if (drivetrain.iAmDockingLeaveMeAlone) {
					estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				}
				drivetrain.addVisionMeasurement(poseArray, Utils.getCurrentTimeSeconds()-latency, estStdDevs);
			}

			prevPose = poseArray;
		} else {
			prevPose = new Pose2d();
		}
    }
}

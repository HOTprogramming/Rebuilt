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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
	Vector<N3> singleTagStdDevs = VecBuilder.fill(4.5, 4.5, 8);
	Vector<N3> multiTagStdDevs = VecBuilder.fill(.55, .55, 1.5);
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

	public void periodic() {
		if(subTable.getEntry("has_pose").getBoolean(false)){
			poseArray = subscriber.get();
			long numTags = subTable.getEntry("num_tags").getInteger(0);
			tagIds = subTable.getEntry("tag_ids").getNumberArray(new Number[0]);
			avgDist = 0;
			Number ambg = subTable.getEntry("ambiguity").getNumber(0);

			for(Number id : tagIds){
				avgDist += layout.getTagPose(id.intValue()).get().getTranslation().toTranslation2d().
					getDistance(poseArray.getTranslation());
			}

			if(numTags > 0){
				avgDist = avgDist / (double) numTags;
			}

			estStdDevs = (numTags > 1) ? multiTagStdDevs : singleTagStdDevs;

			if (numTags == 1) {
				if (avgDist > 2.5) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				else estStdDevs = singleTagStdDevs;
			} else if (numTags == 2) {
				if (avgDist > 5.5) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				else estStdDevs = multiTagStdDevs;
			} else if (numTags == 3) {
				if (avgDist > 8) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				else if (avgDist > 6.5) estStdDevs = multiTagStdDevs.times(1.5);
				else estStdDevs = multiTagStdDevs;
			} else if (numTags > 3) {
				if (avgDist > 10) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				else if (avgDist > 7.5) estStdDevs = multiTagStdDevs.times(1.5);
				else estStdDevs = multiTagStdDevs;
			}

			estStdDevs = estStdDevs.times(1 + ((avgDist * avgDist / 30.0)));

			if(ambg.doubleValue() > .2){
				estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
			}

			if(poseArray.getX() < 20 && poseArray.getX() > 0 && poseArray.getY() < 20 && poseArray.getY() > 0 && numTags > 0){
				if (drivetrain.iAmDockingLeaveMeAlone) {
					estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				}
				drivetrain.addVisionMeasurement(poseArray, Utils.getCurrentTimeSeconds()-.05, estStdDevs);
			}
		}
    }
}

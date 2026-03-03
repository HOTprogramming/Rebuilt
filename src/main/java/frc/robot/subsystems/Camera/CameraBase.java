
package frc.robot.subsystems.Camera;

import java.util.Optional;

import com.ctre.phoenix6.Utils;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.StructTopic;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.subsystems.Drivetrain.Drive;

public class CameraBase {

	private Drive drivetrain;

	NetworkTable subTable;
	StructTopic<Pose2d> pose2dTopic;
	StructSubscriber<Pose2d> subscriber;
	AprilTagFieldLayout layout =  AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
	edu.wpi.first.math.Vector<N3> stdDevs;
	Pose2d poseArray;
	double avgDist;
	Number[] tagIds;
	double numTags;
	Translation2d prevTranslation;

	public CameraBase(Drive drive, String name) {
		drivetrain = drive;

		NetworkTable hotVisionTable = NetworkTableInstance.getDefault().getTable("HOTVision");

		subTable = hotVisionTable.getSubTable(name + "/apriltag");
		pose2dTopic = subTable.getStructTopic("robot_pose2d", Pose2d.struct);
		subscriber = pose2dTopic.subscribe(new Pose2d());
	}

	public void periodic() {
		if(subTable.getEntry("has_pose").getBoolean(false)){
			poseArray = subscriber.get();
			
			avgDist = 0;

			tagIds = subTable.getEntry("tag_ids").getNumberArray(new Number[0]);
			numTags = (double) subTable.getEntry("num_tags").getInteger(0);

			for(Number id : tagIds){
				avgDist += layout.getTagPose(id.intValue()).get().getTranslation().
					getDistance(poseArray.getPose().getTranslation());
			}

			avgDist = avgDist / numTags;

			if (poseArray.getTranslation().getDistance(prevTranslation) > .25) {
				stdDevs = VecBuilder.fill(20,20, 20);
			} else if (numTags == 1 && avgDist > 3){
				stdDevs = VecBuilder.fill(Double.MAX_VALUE,Double.MAX_VALUE, Double.MAX_VALUE);
			} else if (numTags > 1 && avgDist > 5.5){
				stdDevs = VecBuilder.fill(Double.MAX_VALUE,Double.MAX_VALUE, Double.MAX_VALUE);
			} else {
				if(numTags > 1){
					stdDevs = VecBuilder.fill(1 * avgDist, 1 * avgDist, 10);
				} else {
					stdDevs = VecBuilder.fill(5 * avgDist, 5 * avgDist, 10);
				}
			}

			drivetrain.addVisionMeasurement(poseArray, Utils.getCurrentTimeSeconds()-.02, stdDevs);
			prevTranslation = poseArray.getTranslation();
		}
    }
}
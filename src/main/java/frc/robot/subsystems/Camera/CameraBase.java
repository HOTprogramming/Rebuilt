
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
import frc.robot.subsystems.Drivetrain.Drive;

public class CameraBase {
	private Drive drivetrain;

	NetworkTable subTable;
	StructTopic<Pose2d> pose2dTopic;
	StructSubscriber<Pose2d> subscriber;
	edu.wpi.first.math.Vector<N3> stdDevs;
	AprilTagFieldLayout layout =  AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
	double avgDist;
	Pose3d tagPose3d;
	Pose2d tagPose;
	Pose2d poseArray;
	Pose2d prevCameraPose = new Pose2d();
	Number[] tagIds;
	long numTags;

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
			numTags = subTable.getEntry("num_tags").getInteger(0);

			for(Number id : tagIds){
				tagPose3d = layout.getTagPose(id.intValue()).get();

				tagPose = new Pose2d(tagPose3d.getX(), tagPose3d.getY(), new Rotation2d());

				avgDist += Math.abs(Math.hypot(tagPose.minus(drivetrain.getPose()).getMeasureX().baseUnitMagnitude(), 
				tagPose.minus(drivetrain.getPose()).getMeasureY().baseUnitMagnitude()));
			}

			avgDist = avgDist / (double) numTags;

			if (Math.abs(Math.hypot(poseArray.minus(prevCameraPose).getMeasureX().baseUnitMagnitude(), 
				poseArray.minus(prevCameraPose).getMeasureY().baseUnitMagnitude())) > .3){
				stdDevs = VecBuilder.fill(25, 25, 25);
			} else if (numTags == 1 && avgDist > 3){
				stdDevs = VecBuilder.fill(Double.MAX_VALUE,Double.MAX_VALUE, Double.MAX_VALUE);
			} else if (numTags > 1 && avgDist > 6.5) {
				stdDevs = VecBuilder.fill(Double.MAX_VALUE,Double.MAX_VALUE, Double.MAX_VALUE);
			} else {
				if(numTags > 1){
					stdDevs = VecBuilder.fill(.75 * avgDist, .75 * avgDist, 5);
				} else {
					stdDevs = VecBuilder.fill(4 * avgDist, 4 * avgDist, 10);
				}
			}

			drivetrain.addVisionMeasurement(poseArray, Utils.getCurrentTimeSeconds()-.02, stdDevs);

			prevCameraPose = poseArray;
		}
    }
}

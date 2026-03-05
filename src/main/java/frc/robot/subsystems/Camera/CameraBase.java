
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
import edu.wpi.first.math.Vector;

public class CameraBase {

	private Drive drivetrain;

	NetworkTable subTable;
	StructTopic<Pose2d> pose2dTopic;
	StructSubscriber<Pose2d> subscriber;
	AprilTagFieldLayout layout =  AprilTagFieldLayout.loadField(AprilTagFields.k2026RebuiltAndymark);
	Vector<N3> estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
	Vector<N3> singleTagStdDevs = VecBuilder.fill(5, 5, 5);
	Vector<N3> multiTagStdDevs = VecBuilder.fill(.75, .75, 1);
	Pose2d poseArray;
	double avgDist;
	Number[] tagIds;
	double numTags;
	Translation2d prevTranslation = new Translation2d(0, 0);
	String name;


	public CameraBase(Drive drive, String name) {
		drivetrain = drive;
		this.name = name;

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

			for(Number id : tagIds){
				avgDist += layout.getTagPose(id.intValue()).get().getTranslation().toTranslation2d().
					getDistance(poseArray.getTranslation());
			}

			avgDist = avgDist / (double) subTable.getEntry("num_tags").getInteger(0);

			SmartDashboard.putNumber(name + " Avg dist", avgDist);

			if(subTable.getEntry("num_tags").getInteger(0) == 1){
				if (avgDist > 3) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				else estStdDevs = singleTagStdDevs;
			} else if (subTable.getEntry("num_tags").getInteger(0) > 1){
				if (avgDist > 6) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				else estStdDevs = multiTagStdDevs;
			}

			double distFromRobot = poseArray.getTranslation().getDistance(drivetrain.getPose().getTranslation());

			estStdDevs = estStdDevs.times(1 + ((avgDist * avgDist / 30.0) * (1 + distFromRobot/3.0)));

			drivetrain.addVisionMeasurement(poseArray, Utils.getCurrentTimeSeconds()-.02, estStdDevs);
		}
    }
}
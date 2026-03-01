
package frc.robot.subsystems.Camera;

import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructSubscriber;
import edu.wpi.first.networktables.StructTopic;
import frc.robot.subsystems.Drivetrain.Drive;

public class CameraBase {

	private Drive drivetrain;

	NetworkTable subTable;
	StructTopic<Pose2d> pose2dTopic;
	StructSubscriber<Pose2d> subscriber;

	public CameraBase(Drive drive, String name) {
		drivetrain = drive;

		NetworkTable hotVisionTable = NetworkTableInstance.getDefault().getTable("HOTVision");

		subTable = hotVisionTable.getSubTable(name + "/apriltag");
		pose2dTopic = subTable.getStructTopic("robot_pose2d", Pose2d.struct);
		subscriber = pose2dTopic.subscribe(new Pose2d());
	}

	public void periodic() {

		if(subTable.getEntry("has_pose").getBoolean(false)){
			Pose2d poseArray = subscriber.get();

			edu.wpi.first.math.Vector<N3> stdDevs;
			
			if(subTable.getEntry("num_tags").getInteger(0) > 1){
				stdDevs = VecBuilder.fill(1,1, 5.5);
			} else {
				stdDevs = VecBuilder.fill(2,2, 10);
			}

			drivetrain.addVisionMeasurement(poseArray, Utils.getCurrentTimeSeconds(), stdDevs);
		}
    }
}

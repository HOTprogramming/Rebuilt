
package frc.robot.subsystems.Camera;

<<<<<<< HEAD
import java.util.Optional;
=======
import java.util.Arrays;
>>>>>>> a721243b7ca975bde9cfcef03f69f24138f110a9

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
<<<<<<< HEAD
=======
import frc.robot.RobotState;
import frc.robot.subsystems.Drivetrain.Drivetrain;
>>>>>>> a721243b7ca975bde9cfcef03f69f24138f110a9
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
<<<<<<< HEAD
	Vector<N3> singleTagStdDevs = VecBuilder.fill(5, 5, 7.5);
	Vector<N3> multiTagStdDevs = VecBuilder.fill(.5, .5, 2);
=======
	Vector<N3> singleTagStdDevs = VecBuilder.fill(4, 4, 8);
	Vector<N3> twoTagStDevs = VecBuilder.fill(.6, .6, 1);
	Vector<N3> threeTagStDevs = VecBuilder.fill(.55, .55, 1);
	Vector<N3> fourTagStdDevs = VecBuilder.fill(.5, .5, 1);
>>>>>>> a721243b7ca975bde9cfcef03f69f24138f110a9
	Pose2d poseArray;
	double avgDist;
	Number[] tagIds;
	double numTags;
	Translation2d prevTranslation = new Translation2d(0, 0);
	String name;
	Pose2d prevPose = new Pose2d();


	public CameraBase(Drive drive, String name) {
		drivetrain = drive;
		this.name = name;

		NetworkTable hotVisionTable = NetworkTableInstance.getDefault().getTable("HOTVision");

		subTable = hotVisionTable.getSubTable(name + "/apriltag");
		pose2dTopic = subTable.getStructTopic("robot_pose2d", Pose2d.struct);
		subscriber = pose2dTopic.subscribe(new Pose2d());
	}

	public int[] targetTags = {27, 18, 25, 26, 24, 21, 29, 30};

	public void periodic() {
		if(subTable.getEntry("has_pose").getBoolean(false)){
			double latency = subTable.getEntry("latency").getNumber(0).floatValue() / 1000.0;
			poseArray = subscriber.get();
			
			avgDist = 0;
<<<<<<< HEAD

			tagIds = subTable.getEntry("tag_ids").getNumberArray(new Number[0]);
=======
			Number ambg = subTable.getEntry("ambiguity").getNumber(0);
			boolean targeting = false;
>>>>>>> a721243b7ca975bde9cfcef03f69f24138f110a9

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

			avgDist = avgDist / (double) subTable.getEntry("num_tags").getInteger(0);

<<<<<<< HEAD
			SmartDashboard.putNumber(name + " Avg dist", avgDist);
=======
			estStdDevs = (numTags > 1) ? twoTagStDevs : singleTagStdDevs;
>>>>>>> a721243b7ca975bde9cfcef03f69f24138f110a9

			if(subTable.getEntry("num_tags").getInteger(0) == 1){
				if (avgDist > 2.5) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
				else estStdDevs = singleTagStdDevs;
<<<<<<< HEAD
			} else if (subTable.getEntry("num_tags").getInteger(0) > 1){
				if (subTable.getEntry("num_tags").getInteger(0) > 2){
					if (avgDist > 10) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
					else estStdDevs = multiTagStdDevs;
				} else {
					if (avgDist > 5) estStdDevs = VecBuilder.fill(Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE);
					else estStdDevs = multiTagStdDevs;
				}
			}

			double distFromRobot = poseArray.getTranslation().getDistance(drivetrain.getPose().getTranslation());
			double distFromPrev = 0;
			if (prevPose.getX() != 100){
				distFromPrev = poseArray.getTranslation().getDistance(prevPose.getTranslation());
			}

			SmartDashboard.putNumber("Dist From Robot", distFromRobot);

			if(distFromPrev > .5){
				estStdDevs = VecBuilder.fill(20, 20, 20);
			} else {
				if(subTable.getEntry("num_tags").getInteger(0) > 2){
					estStdDevs = estStdDevs.times(1 + ((avgDist * avgDist / 60.0)));
				} else {
					estStdDevs = estStdDevs.times(1 + ((avgDist * avgDist / 30.0)));
				}
			}

			SmartDashboard.putNumber("DIst from prev", distFromPrev);

			if(avgDist < 10 && distFromPrev < .5 && estStdDevs.get(0) <= 100){
				drivetrain.addVisionMeasurement(poseArray, Utils.getCurrentTimeSeconds()-.02, estStdDevs);
			}

			prevPose = poseArray;
		} else {
			prevPose = new Pose2d(100, 100, new Rotation2d());
=======
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
>>>>>>> a721243b7ca975bde9cfcef03f69f24138f110a9
		}
    }
}
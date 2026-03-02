
package frc.robot.subsystems.Camera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.Drive;

public class Camera extends SubsystemBase {

	CameraBase camera0;
	CameraBase camera1;
	CameraBase camera2;

	public Camera(Drive drive) {
		camera0 = new CameraBase(drive, "camera_0");
		camera1 = new CameraBase(drive, "camera_1");
		camera2 = new CameraBase(drive, "camera_1");
	}

	@Override
	public void periodic() {
		camera0.periodic();
		camera1.periodic();
		camera2.periodic();
    }
}

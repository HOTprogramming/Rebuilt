
package frc.robot.subsystems.camera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.drivetrain.Drivetrain;

public class Camera extends SubsystemBase {

	CameraBase back;
	CameraBase left;
	CameraBase front;
	CameraBase right;

	public Camera(Drivetrain drive) {
		back = new CameraBase(drive, "BACK");
		left = new CameraBase(drive, "LEFT");
		front = new CameraBase(drive, "FRONT");
		right = new CameraBase(drive, "RIGHT");
	}

	@Override
	public void periodic() {
		back.periodic();
		left.periodic();
		front.periodic();
		right.periodic();
    }
}

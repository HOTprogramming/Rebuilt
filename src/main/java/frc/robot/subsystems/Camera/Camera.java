
package frc.robot.subsystems.Camera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.Drive;

public class Camera extends SubsystemBase {

	CameraBase back;
	CameraBase left;
	CameraBase front;

	public Camera(Drive drive) {
		back = new CameraBase(drive, "BACK");
		left = new CameraBase(drive, "LEFT");
		front = new CameraBase(drive, "FRONT");
	}

	@Override
	public void periodic() {
		back.periodic();
		left.periodic();
		front.periodic();
    }
}

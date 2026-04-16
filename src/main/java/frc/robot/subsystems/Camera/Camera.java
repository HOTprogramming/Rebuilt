package frc.robot.subsystems.Camera;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Drivetrain.Drivetrain;

public class Camera extends SubsystemBase {

	// CameraBase backLeft;
	// CameraBase backRight;
	CameraBase left;
	CameraBase front;
	CameraBase back;

	public Camera(Drivetrain drive) {
		// backLeft = new CameraBase(drive, "BACK_LEFT");
		// backRight = new CameraBase(drive, "BACK_RIGHT");
		left = new CameraBase(drive, "LEFT");
		front = new CameraBase(drive, "FRONT");
		back = new CameraBase(drive, "BACK");

	}

	@Override
	public void periodic() {
		// backLeft.periodic();
		// backRight.periodic();
		left.periodic();
		front.periodic();
		back.periodic();
    }
}

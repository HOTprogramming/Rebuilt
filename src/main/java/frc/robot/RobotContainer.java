// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystems.Camera.Camera;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveKraken;
import frc.robot.subsystems.Indexer.Index;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Intake.IntakeConstants;

public class RobotContainer {
	SendableChooser<String> Autos = new SendableChooser<>();
	private Command autoCommand;
	private String autoString = "";

	private final Drive drivetrain;
	private final Camera camera;
	private final Index indexer;
	private final Climber climber;
	private final RobotState manager;
	private final Intake intake;

	private final CommandXboxController driver = new CommandXboxController(0);
	private final CommandXboxController operator = new CommandXboxController(1);

	Optional<Alliance> Ally = DriverStation.getAlliance();
	String autoSelect;
	String colorSelect;
	String prevAuto = "";

	public RobotContainer() {
		RobotController.setBrownoutVoltage(Constants.brownoutVoltage);

		// IMPORTANT: When adding autons, they must have their name start with either
		// "Blue-" or "Red-". When adding them to the chooser, do not include the
		// "Blue-" or "Red-"
		// in the name. Example: an auton called "Red-Shoot Left" should be added with a
		// blue version called "Blue-Shoot Left", and type "Autos.addOption("Shoot
		// Left","Shoot Left");"
		Autos.setDefaultOption("Path", "Path");

    Autos.addOption("First Camera Test Auto", "First Camera Test Auto");
    Autos.addOption("NEUTRALSTEAL", "NEUTRALSTEAL");
    Autos.addOption("Grid Test", "Grid Test");
    Autos.addOption("SNOWBLOW", "SNOWBLOW");
	Autos.addOption("NOCLIMB_HalfCourtSteal", "NOCLIMB_HalfCourtSteal");
	Autos.addOption("CLIMB_HalfCourtSteal", "CLIMB_HalfCourtSteal");
	Autos.addOption("NOCLIMB_HalfCourtSteal3", "NOCLIMB_HalfCourtSteal3");
	Autos.addOption("Triple T", "Triple T");
	
    
    
    SmartDashboard.putData("Autos",Autos);

		drivetrain = new Drive(new DriveKraken());
		camera = new Camera(drivetrain);
		indexer = new Index();
		climber = new Climber();
		manager = new RobotState();
		intake = new Intake();

		NamedCommands.registerCommand("AlignPole", drivetrain.alignToPole());
		NamedCommands.registerCommand("OTF", drivetrain.runPathFinding(new Pose2d(8, 4, Rotation2d.fromDegrees(0))));

		NamedCommands.registerCommand("AlignPole", drivetrain.alignToPole());
		NamedCommands.registerCommand("OTF", drivetrain.runPathFinding(new Pose2d(8, 4, Rotation2d.fromDegrees(0))));
		NamedCommands.registerCommand("intake", intake.extend());
		NamedCommands.registerCommand("intake off", intake.retract());
		//Start autoaim at start of auton
		// NamedCommands.registerCommand("auto aim", manager.autoAim());
		NamedCommands.registerCommand("shoot", indexer.indexOnCommand());
		NamedCommands.registerCommand("shoot off", indexer.indexOffCommand());

		configureBindings();
	}

	private void configureBindings() {
		// Drivetrain Controls
		drivetrain.setDefaultCommand(drivetrain
			.run(() -> drivetrain.teleopDrive(-driver.getLeftY(), -driver.getLeftX(), -driver.getRightX())));

		//Robot Centric Drive
		driver.rightBumper().whileTrue(drivetrain.run(() -> drivetrain.robotCentricTeleopDrive(-driver.getLeftY(), -driver.getLeftX(), -driver.getRightX())));
	
		//Slow Mode
		driver.leftBumper().whileTrue(drivetrain.run(() -> drivetrain.teleopDriveSlow(-driver.getLeftY(), -driver.getLeftX(), -driver.getRightX())));

		// Aim
		driver.a().whileTrue(drivetrain.run(() -> drivetrain.aim(-driver.getLeftY(), -driver.getLeftX())));

		// Rotate for Bump
		driver.b().whileTrue(drivetrain.run(() -> {
			double rot = drivetrain.getPose().getRotation().getDegrees();
			double fact;
			fact = Math.abs(rot) > 90 ? 3 : 1;
			if (rot < 0)
				fact *= -1;

			drivetrain.lockRotation(Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY()
				: 0, Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0, new Rotation2d(Math.PI / 4 * fact));
		}));
    
		// X Swerve Drive
		driver.x().whileTrue(drivetrain.xLockCommand());

		//Shoot
		operator.rightTrigger().onTrue(manager.autoAim());
		
		// Shooter Controls (Operator)
		operator.a().onTrue(manager.shooter.goToIdle());
		operator.b().onTrue(manager.shooter.shooterOff());
		operator.y().onTrue(manager.shooter.setHoodAngle(0));

		// Indexer Controls
		driver.rightTrigger().whileTrue(indexer.indexOnCommand()).whileFalse(indexer.indexOffCommand());

		//Align to Pole
		operator.x().whileTrue(drivetrain.alignToPole());

		// Climber Controls
		climber.setDefaultCommand(climber
			.run(() -> climber.m_ClimbKraken.runManual(operator.getLeftX(), operator.getRightX())));

		// Intake Controls
		intake.setDefaultCommand(intake.agitateRetract());
		new Trigger(intake.isAgitateRetracted()).onTrue(intake.stop()).onFalse(intake.run(() -> intake.mIntakeKraken.runVelocity(IntakeConstants.kIntakeSpeed)));
		operator.rightBumper().toggleOnTrue(intake.extend());
		operator.leftBumper().onTrue(intake.agitate());
		operator.leftTrigger().toggleOnTrue(intake.rollerStop());
	}

	public void updateAutonCommand() {
		Ally = DriverStation.getAlliance();

		if (!Ally.isPresent()) {
			colorSelect = "Red-";
		} else {
			if (Ally.get() == Alliance.Red) {
				colorSelect = "Red-";
			} else {
				colorSelect = "Blue-";
			}
		}

		autoSelect = colorSelect + Autos.getSelected();

		if (!autoSelect.equals(prevAuto)) {
			autoCommand = new PathPlannerAuto(autoSelect);

			try {
				PathPlannerAuto.getPathGroupFromAutoFile(autoSelect);
			} catch (IOException e) {
				System.err.println("No Auto File: " + e.toString());
			} catch (ParseException e) {
				System.err.println("idk good luck: " + e.toString());
			} catch (IndexOutOfBoundsException e) {
				System.err.println("no auto Paths: " + e.toString());
			}
		}

		prevAuto = autoSelect;
	}

	public Command getAutonomousCommand() {
		return autoCommand;
	}
}
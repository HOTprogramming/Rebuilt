// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import static edu.wpi.first.units.Units.Inches;

import java.io.IOException;
import java.util.Optional;
import java.util.function.Supplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Camera.Camera;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveKraken;
import frc.robot.subsystems.Indexer.Index;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.RobotState; 

public class RobotContainer {
	SendableChooser<String> Autos = new SendableChooser<>();
	private Command autoCommand;
	private String autoString = "";

	private final Drive drivetrain;
	private final Camera camera;
	public final Index indexer;
	private final Climber climber;
	public final RobotState manager;
	public final Intake intake;


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

		Autos.addOption("ohno", "ohno");
		Autos.addOption("FunkyCheck", "FunkyCheck");
		Autos.addOption("RightClimb", "Climb");
		Autos.addOption("LeftClimb", "LeftClimb");

		SmartDashboard.putData("Autos", Autos);

		drivetrain = new Drive(new DriveKraken());
		camera = new Camera(drivetrain);
		indexer = new Index();
		climber = new Climber();
		manager = new RobotState();
		intake = new Intake();

		NamedCommands.registerCommand("align pole", drivetrain.driveToPoleCoordinate());
		NamedCommands.registerCommand("align pole autoflip", drivetrain.driveToPoleCoordinate_autoflip());
		NamedCommands.registerCommand("OTF", drivetrain.runPathFinding(new Pose2d(8, 4, Rotation2d.fromDegrees(0))));
		NamedCommands.registerCommand("intake", intake.extend());
		NamedCommands.registerCommand("intake off", intake.teleopRetract());
		NamedCommands.registerCommand("intake agitate", intake.agitate());

		//Start autoaim at start of auton
		// NamedCommands.registerCommand("auto aim", manager.autoAim());
		NamedCommands.registerCommand("shoot", indexer.indexOnCommand());
		NamedCommands.registerCommand("shoot off", indexer.indexOffCommand());
		NamedCommands.registerCommand("autoclimb",climber.autoClimb());
		NamedCommands.registerCommand("hood down", Commands.sequence(manager.shooter.setHoodAngle(0), Commands.runOnce(() -> manager.shooter.mShooterKraken.runVelocity(47))));
		NamedCommands.registerCommand("shooter on",manager.shooter.autoAim());
		NamedCommands.registerCommand("auton spin", manager.shooter.AutonSpin());


		NamedCommands.registerCommand("Auto Aim", 			
			Commands.parallel(
				drivetrain.run(() -> drivetrain.aim(0,0)),
					manager.shooter.autoAim(),
					Commands.sequence(
						Commands.run(() -> {}).until(() -> manager.shooter.readyToShoot() && drivetrain.isAtTargetAngleAuton()),
						indexer.indexOnCommand(),
						
					Commands.repeatingSequence(
						intake.agitateRetract(),
						new WaitCommand(.35),
						intake.extend(),
						new WaitCommand(.35)
					)
				)
			));

		NamedCommands.registerCommand("Auto Aim test", 			
			Commands.parallel(
					manager.shooter.autoAim(),
					Commands.sequence(
						Commands.run(() -> {}).until(() -> manager.shooter.readyToShoot()),
						indexer.indexOnCommand(),
						
					Commands.repeatingSequence(
						intake.agitateRetract(),
						new WaitCommand(.35),
						intake.extend(),
						new WaitCommand(.35)
					)
				)
			));

		NamedCommands.registerCommand("Stop Shooting", 
				Commands.parallel(
					indexer.indexOffCommand(),
					Commands.sequence(
						manager.shooter.dodgeTrench()
					)
			));

		configureBindings();
	}

	
	private void configureBindings() {

		//DRIVER CONTROLS

		// Drivetrain Controls
		drivetrain.setDefaultCommand(drivetrain
			.run(() -> drivetrain.teleopDrive(-driver.getLeftY(), -driver.getLeftX(), -driver.getRightX())));

		//Robot Centric Drive
		//driver.rightBumper().whileTrue(drivetrain.run(() -> drivetrain.robotCentricTeleopDrive(-driver.getLeftY(), -driver.getLeftX(), -driver.getRightX())));
		driver.rightBumper().onTrue(drivetrain.alignToPole());

		driver.a().onTrue(drivetrain.driveToPoleCoordinate());
		driver.b().onTrue(Commands.sequence(drivetrain.angleSwerveWheels(), drivetrain.driveToPoleCoordinate_autoflip()));
		// driver.rightBumper().onTrue(
		// 	Commands.sequence(
		// 		drivetrain.alignToPole(),
		// 		Commands.waitSeconds(0.1),
		// 		drivetrain.dockToLadder())
		// 	);		
		
		//Slow Mode
		//driver.leftBumper().whileTrue(drivetrain.run(() -> drivetrain.teleopDriveSlow(-driver.getLeftY(), -driver.getLeftX(), -driver.getRightX())));

		// Aim
		//driver.a().whileTrue(drivetrain.run(() -> drivetrain.aim(-driver.getLeftY(), -driver.getLeftX())));

		// Rotate for Bump
		// driver.b().whileTrue(drivetrain.run(() -> {
		// 	double rot = drivetrain.getPose().getRotation().getDegrees();
		// 	double fact;
		// 	fact = Math.abs(rot) > 90 ? 3 : 1;
		// 	if (rot < 0)
		// 		fact *= -1;

		// 	drivetrain.lockRotation(Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY()
		// 		: 0, Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0, new Rotation2d(Math.PI / 4 * fact));
		// }));

		// X the Wheels
		driver.x().whileTrue(drivetrain.xLockCommand());

		// Align to trench (Hood will go down)
		driver.y().whileTrue(drivetrain.driveToTrenchY(() -> -driver.getLeftY()));	

		// Indexer Controls (aka shoot)
		// driver.rightTrigger().whileTrue(indexer.indexOnCommand()).whileFalse(indexer.indexOffCommand());
		driver.rightTrigger().whileTrue(
			Commands.parallel(
				drivetrain.run(() -> drivetrain.aim(-driver.getLeftY(), -driver.getLeftX())).unless(manager::isManualMode),
				Commands.either(
					manager.shooter.autoAim(),
					manager.shooter.holdTarget(),
					manager::isNormalMode),
				Commands.sequence(
					Commands.run(() -> {}).until(() -> manager.shooter.readyToShoot() && (drivetrain.isAtTargetAngle() || manager.isManualMode())),
					indexer.indexOnCommand(),
					Commands.repeatingSequence(
						intake.agitateRetract(),
						new WaitCommand(.25),
						intake.extendShort(),
						new WaitCommand(.25)
					).until(() -> driver.leftTrigger().getAsBoolean())
				)	
			)
		).onFalse(
			Commands.parallel(
				Commands.sequence(
					indexer.indexOffCommand(),
					manager.shooter.dodgeTrench()
				)
			)
		).onFalse(intake.stop());

		//Shooter
		// manager.shooter.setDefaultCommand(manager.idleCommands());
		operator.rightBumper().onTrue(manager.shooter.idle());

		//Intake
		// driver.leftTrigger().whileTrue(intake.extend()).onFalse(intake.completeStop());

		driver.leftTrigger().onTrue(
			Commands.runOnce(() -> intake.shouldStop = false)
		).whileTrue(
			Commands.repeatingSequence(
				Commands.runOnce(() -> {
					if(intake.IsIntakeExtended() || intake.mIntakeKraken.m_ExtendKraken.motor.getSupplyCurrent().getValueAsDouble() > 80.0){
		
						intake.shouldStop = true;
					}
				}),
				Commands.either(intake.stopButSpin(), intake.extend(), () -> intake.shouldStop)
			)
		).whileFalse(intake.completeStop())
		.onFalse(
			Commands.repeatingSequence(
				intake.agitateRetract(),
				new WaitCommand(.25),
				intake.extendShort(),
				new WaitCommand(.25)
			).onlyIf(() -> driver.rightTrigger().getAsBoolean()));

		operator.leftBumper().onTrue(intake.teleopRetract());

		//Climb Mode
		operator.start().onTrue(
			Commands.parallel(
				Commands.runOnce(() -> manager.setClimbMode()),
				intake.fullretract(),
				manager.shooter.shooterOff()
			)); 

		//Normal Mode
		operator.back().onTrue(Commands.runOnce(() -> manager.setNormalMode())); 

		//Shot Map: Deploy Climber
		operator.x().onTrue(
    	Commands.either(
		climber.fullDeploy(), 
		Commands.runOnce(() -> manager.setNormalMode()),
		//manager.idleCommands(),
        manager::isClimbMode));

		//Climber Pole Preset: Step 1 of Climb
		operator.a().onTrue(
    	Commands.either(
			Commands.parallel(
				drivetrain.driveTowardsClimberSide(0.5).withTimeout(1), 
				Commands.sequence(Commands.waitSeconds(0.5), climber.stageThreeClimb())
			),
			Commands.sequence(Commands.runOnce(() -> manager.setManualMode()), manager.shooter.PresetClimberPole()),
			manager::isClimbMode
		));

		//Trench Preset: Step 2 of Climb
		operator.b().onTrue(
    	Commands.either(
		climber.stow(), 
		Commands.sequence(Commands.runOnce(() -> manager.setManualMode()), manager.shooter.PresetTrench()),
        manager::isClimbMode));

		//Batter Shot Preset : Step 3 of Climb
		operator.y().onTrue(
    	Commands.either(
		climber.teleopDeploy(), 
		Commands.sequence(Commands.runOnce(() -> manager.setManualMode()), manager.shooter.PresetBatter()),
        manager::isClimbMode));

		//Agitate
		operator.rightTrigger().whileTrue(intake.agitate());

		//Intake Roller Stop
		operator.leftTrigger().whileTrue(intake.rollerStop());

		operator.povUp().onTrue(Commands.runOnce(() -> {
			manager.shooter.shotMapOffset += 0.1524;
		}));

		operator.povDown().onTrue(Commands.runOnce(() -> {
			manager.shooter.shotMapOffset -= 0.1524;
		}));


		//Climber Controls
		climber.setDefaultCommand(climber
			.run(() -> climber.m_ClimbKraken.runManual(operator.getLeftX(), operator.getRightX()))
			.onlyIf(manager::isClimbMode));
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
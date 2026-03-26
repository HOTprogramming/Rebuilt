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
		theta.enableContinuousInput(-Math.PI, Math.PI);
		
		NamedCommands.registerCommand("align pole autoflip", Commands.sequence(drivetrain.angleSwerveWheels(), drivetrain.driveToPoleCoordinate_autoflip()));
		NamedCommands.registerCommand("auto undock", Commands.sequence(climber.deploy(),drivetrain.driveTowardsClimberSide(-1.5).withTimeout(0.8)));
		NamedCommands.registerCommand("Intake Out", intake.extend());
		NamedCommands.registerCommand("deploy climber", climber.autonDeploy());
		NamedCommands.registerCommand("zero wrist", climber.zeroSpinner());
		NamedCommands.registerCommand("Intake Stop", intake.completeStop());
		NamedCommands.registerCommand("Shoot", Commands.parallel(indexer.indexOnCommand(), intake.packingAgitate()));
		NamedCommands.registerCommand("Aim", 
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

		autoChooser = AutoBuilder.buildAutoChooser();

		SmartDashboard.putData("Auto Chooser", autoChooser);

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());        
		CommandScheduler.getInstance().schedule(PathfindingCommand.warmupCommand());

    }

    private void configureBindings() {
        drivetrain.setDefaultCommand(
            drivetrain.run(() -> drivetrain.teleopDrive(
                -driver.getLeftY(),
                -driver.getLeftX(),
                -driver.getRightX()
            ))
        );

		shooter.setDefaultCommand(shooter.hoodDown());

		driver.rightTrigger().whileTrue(drivetrain.run(() -> drivetrain.aim(-driver.getLeftY(), -driver.getLeftX()))).whileTrue(shooter.autoAim())
		.whileTrue(
			Commands.waitUntil(() -> shooter.readyToShoot() && drivetrain.isAtTargetAngle()).andThen(Commands.parallel(
				indexer.indexOnCommand(),
				intake.packingAgitate().unless(driver.leftTrigger()))
		)).onFalse(indexer.indexOffCommand())
		.onFalse(intake.completeStop().unless(driver.leftTrigger()));

		driver.leftTrigger().whileTrue(intake.extend()).onFalse(intake.completeStop()).onFalse(intake.packingAgitate().onlyIf(driver.rightTrigger()));
		
		driver.b().onTrue(NamedCommands.getCommand("align pole autoflip"));
	
		operator.leftBumper().onTrue(intake.teleopRetract());

		//Climb Mode
		operator.start().onTrue(
			Commands.parallel(
				Commands.runOnce(() -> manager.setClimbMode()),
				intake.fullretract(),
				manager.shooter.shooterOff()
			)); 

		// Deploy Climber
		operator.x().and(RobotState::isClimbMode).onTrue(climber.zeroSpinner());

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

		RobotModeTriggers.disabled().onTrue(
			Commands.runOnce(() -> {
				shooter.targetSpeed = 0;
				RobotState.isShooting = false;
				shooter.mShooterKraken.stop();
				intake.completeStopMethod();
				indexer.m_IndexKraken.stop();
				climber.m_ClimbKraken.stop();
			}).ignoringDisable(true)
		);

		//Climber Controls
		climber.setDefaultCommand(climber
			.run(() -> climber.m_ClimbKraken.runManual(operator.getLeftX(), operator.getRightX()))
			.onlyIf(RobotState::isClimbMode));

		driver.start().onTrue(Commands.runOnce(() -> intake.mIntakeKraken.m_ExtendKraken.motor.setPosition(0)).ignoringDisable(true));

		drivetrain.registerTelemetry(drivetrain.logger::telemeterize);
    }

	public Command getAutonomousCommand() {
		return autoCommand;
	}
}
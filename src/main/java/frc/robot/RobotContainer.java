package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Camera.Camera;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.DrivetrainTelemetry;
import frc.robot.subsystems.Drivetrain.constants.DrivetrainConstants;
import frc.robot.subsystems.Indexer.Index;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class RobotContainer {
	SendableChooser<String> Autos = new SendableChooser<>();
	private final CommandXboxController driver = new CommandXboxController(0);
	private final CommandXboxController operator = new CommandXboxController(1);

    public final Drivetrain drivetrain = DrivetrainConstants.createDrivetrain();
	public final Camera camera = new Camera(drivetrain);
	public final Index indexer = new Index();
	public final Climber climber = new Climber();
	public final Intake intake = new Intake();
	public final Shooter shooter = new Shooter();

	PIDController theta = new PIDController(5, 0, 0);

	private final SendableChooser<Command> autoChooser;

    public RobotContainer() {
		RobotController.setBrownoutVoltage(Constants.brownoutVoltage);
		theta.enableContinuousInput(-Math.PI, Math.PI);
		
		NamedCommands.registerCommand("align pole autoflip", Commands.sequence(drivetrain.angleSwerveWheels(), drivetrain.driveToPoleCoordinate_autoflip()));
		NamedCommands.registerCommand("auto undock", Commands.sequence(climber.deploy(),drivetrain.driveTowardsClimberSide(-1.5).withTimeout(0.8)));
		NamedCommands.registerCommand("Intake Out", intake.extend());
		NamedCommands.registerCommand("deploy climber", climber.teleopDeploy());
		NamedCommands.registerCommand("zero wrist", climber.zeroSpinner());
		NamedCommands.registerCommand("Intake Stop", intake.completeStop());
		NamedCommands.registerCommand("Shoot", Commands.parallel(indexer.indexOnCommand(), intake.packingAgitate()));
		NamedCommands.registerCommand("Aim", 
			Commands.parallel(
				 Commands.run(() -> 
				 drivetrain.aim(0,0)), 
				 shooter.autoAim()));
		NamedCommands.registerCommand("auto climb", climber.autoClimb());
		
		NamedCommands.registerCommand("Aim Override", Commands.runOnce(() -> PPHolonomicDriveController.overrideRotationFeedback(
				() -> theta.calculate(drivetrain.getState().Pose.getRotation().getRadians(), RobotState.turretAngle))));
		NamedCommands.registerCommand("Stop Shooting", 
			Commands.parallel(
				indexer.indexOffCommand(),
				shooter.hoodDown()	
			)
		);
		NamedCommands.registerCommand("Go To Shoot", shooter.autoAim());
		NamedCommands.registerCommand("Agitate", intake.packingAgitate());
		NamedCommands.registerCommand("Run Shoot", Commands.parallel(NamedCommands.getCommand("Aim"), NamedCommands.getCommand("Shoot")));

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
		.onFalse(intake.completeStop());

		driver.leftTrigger().whileTrue(intake.extend()).onFalse(intake.completeStop()).onFalse(intake.packingAgitate().onlyIf(driver.rightTrigger()));
		
		driver.b().onTrue(NamedCommands.getCommand("align pole autoflip"));
	
		operator.leftBumper().onTrue(intake.teleopRetract());
		operator.rightBumper().onTrue(intake.packingAgitate());
		//Temp button to trigger the L1 climb sequence
		operator.leftTrigger().onTrue(climber.autoClimb());
		// Undock from the pole after climbing
		operator.rightTrigger().whileTrue(NamedCommands.getCommand("auto undock"));
		// Stow climber after undocking
		operator.rightBumper().onTrue(climber.stow());

		operator.back().onTrue(Commands.runOnce(() -> RobotState.setNormalMode())); 

		operator.start().onTrue(
			Commands.parallel(
				Commands.runOnce(() -> RobotState.setClimbMode()),
				intake.fullRetract(),
				shooter.stop()
			)); 

		// Deploy Climber
		operator.x().and(RobotState::isClimbMode).onTrue(climber.fullDeploy());

		//Climber Pole Preset: Step 1 of Climb
		operator.a().onTrue(
    	Commands.either(
			Commands.parallel(
				drivetrain.driveTowardsClimberSide(0.5).withTimeout(1), 
				Commands.sequence(Commands.waitSeconds(0.5), climber.stageThreeClimb())
			),
			shooter.PresetClimberPole(),
			RobotState::isClimbMode
		));

		//Trench Preset: Step 2 of Climb
		operator.b().onTrue(
    	Commands.either(
		climber.stow(), 
		shooter.PresetTrench(),
        RobotState::isClimbMode));

		//Batter Shot Preset : Step 3 of Climb
		operator.y().onTrue(
    	Commands.either(
		climber.teleopDeploy(), 
		shooter.PresetBatter(),
        RobotState::isClimbMode));

		RobotModeTriggers.disabled().onTrue(
			Commands.runOnce(() -> {
				shooter.targetSpeed = 0;
				shooter.mShooterKraken.stop();
				intake.mIntakeKraken.stop();
				indexer.m_IndexKraken.stop();
				climber.m_ClimbKraken.stop();
			}).ignoringDisable(true)
		);

		//Climber Controls
		climber.setDefaultCommand(climber
			.run(() -> climber.m_ClimbKraken.runManual(operator.getLeftX(), operator.getRightX()))
			.onlyIf(RobotState::isClimbMode));

		driver.start().onTrue(Commands.runOnce(() -> intake.mIntakeKraken.m_ExtendKraken.motor.setPosition(0)).ignoringDisable(true));
    }

    public Command getAutonomousCommand() {
    	return autoChooser.getSelected();
    }
}
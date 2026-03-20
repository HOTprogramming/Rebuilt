package frc.robot;

import java.util.Optional;

import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import frc.robot.subsystems.Camera.Camera;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.Drivetrain;
import frc.robot.subsystems.Drivetrain.constants.DrivetrainConstants;
import frc.robot.subsystems.Indexer.Index;
import frc.robot.subsystems.Intake.Intake;
import frc.robot.subsystems.Shooter.Shooter;

public class RobotContainer {
	private final CommandXboxController driver = new CommandXboxController(0);
	private final CommandXboxController operator = new CommandXboxController(1);

    public final Drivetrain drivetrain = DrivetrainConstants.createDrivetrain();
	public final Camera camera = new Camera(drivetrain);
	public final Index indexer = new Index();
	public final Climber climber = new Climber();
	public final Intake intake = new Intake();
	public final Shooter shooter = new Shooter();

	PIDController theta = new PIDController(7, 0, 0);

    public RobotContainer() {
		theta.enableContinuousInput(-Math.PI, Math.PI);
		
		NamedCommands.registerCommand("Intake Out", intake.extend());
		NamedCommands.registerCommand("Intake Stop", intake.completeStop());
		NamedCommands.registerCommand("Shoot", indexer.indexOnCommand());
		NamedCommands.registerCommand("Aim", Commands.run(() -> drivetrain.aim(0, 0)));
		NamedCommands.registerCommand("Aim Override", 
			Commands.runOnce(() -> PPHolonomicDriveController.overrideRotationFeedback(
				() -> theta.calculate(drivetrain.getState().Pose.getRotation().getRadians(), RobotState.turretAngle + Math.PI))
			));
		NamedCommands.registerCommand("Cancel Override", 
			Commands.runOnce(() -> PPHolonomicDriveController.clearRotationFeedbackOverride())
		);

		
		NamedCommands.registerCommand("Go To Shoot", shooter.autoAim());

        configureBindings();

        CommandScheduler.getInstance().schedule(FollowPathCommand.warmupCommand());
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

		operator.leftBumper().onTrue(intake.teleopRetract());

		operator.back().onTrue(Commands.runOnce(() -> RobotState.setNormalMode())); 

		operator.start().onTrue(
			Commands.parallel(
				Commands.runOnce(() -> RobotState.setClimbMode()),
				intake.fullretract(),
				shooter.stop()
			)); 

		// Deploy Climber
		operator.x().and(RobotState::isClimbMode).onTrue(climber.fullDeploy());

		//Climber Pole Preset: Step 1 of Climb
		operator.a().whileTrue(
    	Commands.either(
			Commands.parallel(
				// drivetrain.driveTowardsClimberSide(0.5).withTimeout(1), 
				Commands.sequence(Commands.waitSeconds(0.5), climber.stageThreeClimb())
			),
			shooter.PresetClimberPole(),
			RobotState::isClimbMode
		));

		//Trench Preset: Step 2 of Climb
		operator.b().whileTrue(
    	Commands.either(
		climber.stow(), 
		shooter.PresetTrench(),
        RobotState::isClimbMode));

		//Batter Shot Preset : Step 3 of Climb
		operator.y().whileTrue(
    	Commands.either(
		climber.teleopDeploy(), 
		shooter.PresetBatter(),
        RobotState::isClimbMode));

		RobotModeTriggers.disabled().onTrue(
			Commands.run(() -> {
				shooter.targetSpeed = 0;
				shooter.mShooterKraken.stop();
				intake.mIntakeKraken.stop();
				indexer.m_IndexKraken.stop();
			}).ignoringDisable(true)
		);
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Test");
    }
}
package frc.robot;

import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Camera.Camera;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drivetrain.Drivetrain;
import frc.robot.subsystems.drivetrain.constants.DrivetrainConstants;
import frc.robot.subsystems.indexer.Index;
import frc.robot.subsystems.intake.Intake;

public class RobotContainer {
	private final CommandXboxController driver = new CommandXboxController(0);
	private final CommandXboxController operator = new CommandXboxController(1);

    public final Drivetrain drivetrain = DrivetrainConstants.createDrivetrain();
    public final Camera camera = new Camera(drivetrain);
	public final Index indexer = new Index();
	public final Climber climber = new Climber();
	public final RobotState state = new RobotState();
	public final Intake intake = new Intake();

    public RobotContainer() {
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

        driver.leftBumper().onTrue(drivetrain.runOnce(drivetrain::seedFieldCentric));
    }

    public Command getAutonomousCommand() {
        return new PathPlannerAuto("Test");
    }
}
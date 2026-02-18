// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.io.IOException;
import java.util.Optional;
import java.util.function.DoubleSupplier;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.subsystems.Camera.Camera;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Drivetrain.DriveKraken;
import frc.robot.subsystems.Indexer.Index;
import frc.robot.subsystems.Intake.Intake;

public class RobotContainer {
  SendableChooser<String> Autos = new SendableChooser<>();
  private Command autoCommand;
  private String autoString;

  private final Drive drivetrain;
  private final Camera camera;
  private final Shooter shooter;
  private final Index indexer;
  private final Climber climber;
  private final Intake intake;

  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    RobotController.setBrownoutVoltage(Constants.brownoutVoltage); // stops stuttering under high load when the battery is good.

    //IMPORTANT: When adding autons, they must have their name start with either "Blue-" or "Red-". When adding them to the chooser, do not include the "Blue-" or "Red-"
    //in the name. Example: an auton called "Red-Shoot Left" should be added with a blue version called "Blue-Shoot Left", and type "Autos.addOption("Shoot Left","Shoot Left");"
    Autos.setDefaultOption("Path", "Path");

    Autos.addOption("First Camera Test Auto", "First Camera Test Auto");
    Autos.addOption("NEUTRALSTEAL", "NEUTRALSTEAL");
    Autos.addOption("Grid Test", "Grid Test");
    Autos.addOption("SNOWBLOW", "SNOWBLOW");
    
    SmartDashboard.putData("Autos",Autos);

    drivetrain = new Drive(new DriveKraken());
    camera = new Camera(drivetrain);
    shooter = new Shooter();
    indexer = new Index();
    climber = new Climber();
	intake = new Intake();

    configureBindings();

    //NamedCommands.registerCommand("Test Override", drivetrain.testOverride()); 
  }

  private void configureBindings() {   

    // Drivetrain Controls
    drivetrain.setDefaultCommand(drivetrain.run(() -> 
      drivetrain.teleopDrive(
        Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY() : 0,
        Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0,
        Math.abs(driver.getRightX()) >= 0.015 ? -driver.getRightX() : 0
    )));

    driver.a().whileTrue(drivetrain.run(()->
      drivetrain.aim(
        Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY() : 0,
        Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0
    )));

    driver.b().whileTrue(drivetrain.run(()-> {
      double rot = drivetrain.getPose().getRotation().getDegrees(); //robot angle, from -180 to 180
      double fact; //factor to multiply PI/4 radians by to get the nearest 45 degree angle.
      if (Math.abs(rot) > 90) { //This way of calculating fact reduces if statements.
        fact = 3; //The absolute value of the correct angle is 3PI/4 (or 135 degrees), so fact is 3
      } else {
        fact = 1; //The absolute value of the correct angle is PI/4 (or 45 degrees), so fact is 1
      }

      if (rot < 0) {
        fact *= -1; //Correct side of the robot
      }
      
      drivetrain.lockRotation(Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY() : 0,
      Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0,
      new Rotation2d(Math.PI/4 * fact));
    }));

    driver.x() .whileTrue(
        AutoBuilder.pathfindToPose(
            new Pose2d(8.296, 4.061, Rotation2d.fromDegrees(0)), 
            new PathConstraints(5.0, 3.0, 3.14, 6.28), 
            0.0 // Goal end velocity
        )
    );

    //translate to Y target on button press
     driver.y().whileTrue(drivetrain.run(()->
      drivetrain.fixedY(Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY() : 0,
        new Rotation2d(3.142)
    )).beforeStarting(drivetrain.resetControllers()));

    // Shooter Controls

    operator.povUp().onTrue(shooter.shootSpeedUp());
    operator.povDown().onTrue(shooter.shootSpeedDown());
    operator.a().onTrue(shooter.goToIdle());
    operator.x().onTrue(shooter.goToShoot());
    operator.b().onTrue(shooter.shooterOff());

    // Indexer Controls
    operator.rightTrigger().whileTrue(indexer.indexOnCommand()).whileFalse(indexer.indexOffCommand());

    // Climber Controls
    climber.setDefaultCommand(climber.run(() -> climber.m_ClimbKraken.runManual(operator.getLeftX(), operator.getRightX())));

	// Intake Controls
	operator.leftTrigger().whileTrue(intake.extend()).whileFalse(intake.retract());

    NamedCommands.registerCommand("OTF", drivetrain.runOnTheFly());
  }

  public void updateAutonCommand() {
    Optional<Alliance> Ally = DriverStation.getAlliance();
    String AutoSelect;
    String ColorSelect;
    if (!Ally.isPresent()) {
      ColorSelect = "Red-";
    } else {
      if (Ally.get() == Alliance.Red) {
        ColorSelect = "Red-";
      } else {
        ColorSelect = "Blue-";
      }
    }
    AutoSelect = ColorSelect + Autos.getSelected();
    if (autoString != null) {
      if (AutoSelect != autoString) {
        autoString = AutoSelect;
        autoCommand = new PathPlannerAuto(autoString);
        try {
          PathPlannerAuto.getPathGroupFromAutoFile(autoString);
        } catch (IOException e) {
          System.err.println("No Auto File: " + e.toString());
        } catch (ParseException e) {
          System.err.println("idk good luck: " + e.toString());
        } catch (IndexOutOfBoundsException e) {
          System.err.println("no auto Paths: " + e.toString());
        }
      }
    } else {
      autoString = AutoSelect;
    }
  }


  public Command getAutonomousCommand() {
    return autoCommand;
  }
}
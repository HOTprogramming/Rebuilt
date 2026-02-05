// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.*;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Camera.Camera;
import frc.robot.subsystems.Climber.Climber;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Drivetrain.DriveKraken;
import frc.robot.subsystems.Indexer.Index;



public class RobotContainer {

  SendableChooser<String> Autos = new SendableChooser<>();
  private Command autoCommand;
  private String autoString;

  private Drive drivetrain;
  private Camera cameraSubsystem;
  private final Shooter m_shooter;
  private final Climber m_climber;
  private final Index m_indexer;

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

    SmartDashboard.putData("Autos",Autos);

    drivetrain = new Drive(new DriveKraken());
    cameraSubsystem = new Camera(drivetrain);
    m_shooter = new Shooter();
    m_climber = new Climber();
    m_indexer = new Index();

    configureBindings();
  }

  private void configureBindings() {   
      drivetrain.setDefaultCommand
      (drivetrain.run(() -> {
        drivetrain.teleopDrive(
          Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0,
          Math.abs(driver.getRightX()) >= 0.015 ? -driver.getRightX() : 0);
        }
      ));

      driver.a().whileTrue(drivetrain.run(()->
      drivetrain.facePoint(Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0,
          new Pose2d(11.91,4.03,Rotation2d.fromDegrees(0))))
      );

      driver.start().whileTrue(drivetrain.run(()->
        drivetrain.resetPidgeon())
      );

      driver.b().whileTrue(drivetrain.run(()->
      drivetrain.lockRotation(Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY() : 0,
      Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0,
      new Rotation2d(Math.PI/4))));

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
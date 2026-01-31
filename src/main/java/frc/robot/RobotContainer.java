// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.io.IOException;
import java.util.Optional;

import org.json.simple.parser.ParseException;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Shooter.Shooter;
import frc.robot.subsystems.Drivetrain.DriveKraken;

public class RobotContainer {

  SendableChooser<String> Autos = new SendableChooser<>();
  private Command autoCommand;
  private String autoString;

  private Drive drivetrain;
  private Shooter m_shooter;
  
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);

  public RobotContainer() {
    //RobotController.setBrownoutVoltage(Constants.brownoutVoltage); // stops stuttering under high load when the battery is good.

    //IMPORTANT: When adding autons, they must have their name start with either "Blue-" or "Red-". When adding them to the chooser, do not include the "Blue-" or "Red-"
    //in the name. Example: an auton called "Red-Shoot Left" should be added with a blue version called "Blue-Shoot Left", and type "Autos.addOption("Shoot Left","Shoot Left");"
    Autos.setDefaultOption("Auto", "Path");
    SmartDashboard.putData("Autos",Autos);

    drivetrain = new Drive(new DriveKraken());

    m_shooter = new Shooter();

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

    /* For Testing: Delete if needed. */
    //driver.a().toggleOnTrue(m_shooter.goTo15());
    //driver.b().toggleOnTrue(m_shooter.goTo30());
    //driver.x().toggleOnTrue(m_shooter.goTo45());
    //driver.y().toggleOnTrue(m_shooter.goTo60());

    //driver.leftTrigger().toggleOnTrue(m_shooter.goToShoot());
    //driver.rightTrigger().toggleOnTrue(m_shooter.goToIdle());
  }

  public void updateAutonCommand() {
    Optional<Alliance> Ally = DriverStation.getAlliance();
    String AutoSelect;
    String ColorSelect;
    if (!Ally.isPresent()) {
      ColorSelect = "";
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
          drivetrain.setAutonStartPose(PathPlannerAuto.getPathGroupFromAutoFile(autoString).get(0).getStartingDifferentialPose());
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
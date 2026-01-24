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
import frc.robot.subsystems.Drivetrain.DriveKraken;

public class RobotContainer {

  private SendableChooser<String> chooser = new SendableChooser<>();
  private Command autoCommand;
  private String autoString;

  private Drive drivetrain;
  
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  
  SendableChooser<String> SideColor = new SendableChooser<>();
  SendableChooser<String> Autos = new SendableChooser<>();

  public RobotContainer() {
    //RobotController.setBrownoutVoltage(Constants.brownoutVoltage); // stops stuttering under high load when the battery is good.

    drivetrain = new Drive(new DriveKraken());

    Autos.setDefaultOption("Auto", "Auto");
    Autos.addOption("Test", "Test");
    Autos.addOption("Left-Shoot&Climb-Ramp From Neutral", "Left-Shoot&Climb-Ramp From Neutral");
    Autos.addOption("Right-Shoot&Climb-Ramp From Neutral", "Right-Shoot&Climb-Ramp From Neutral");  
    Autos.addOption("Left-Shoot&Climb-Depot&Outpost","Left-Shoot&Climb-Depot&Outpost");
    Autos.addOption("Right-Shoot&Climb-Outpost&Depot","Right-Shoot&Climb-Outpost&Depot");
    Autos.addOption("Center-Shoot&Climb-Outpost", "Center-Shoot&Climb-Outpost");
    Autos.addOption("Right-Scatter-Neutral", "Right-Scatter-Neutral");
    Autos.addOption("Left-Scatter-Neutral", "Left-Scatter-Neutral");
    SmartDashboard.putData("Autos",Autos);

    SideColor.setDefaultOption("None","None");
    SideColor.addOption("Red","Red");
    SideColor.addOption("Blue","Blue");
    SmartDashboard.putData("Side Color",SideColor);

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

      driver.rightTrigger().whileTrue(drivetrain.run(()->
      drivetrain.facePoint(Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0,
          new Pose2d(3.33,3.33,Rotation2d.fromDegrees(0))))
      );

      NamedCommands.registerCommand("OTF", drivetrain.generateOnTheFly());
      NamedCommands.registerCommand("R_OTF", drivetrain.runOnTheFly());

      new EventTrigger("OTF").onTrue(Commands.runOnce(() -> drivetrain.generateOnTheFly()));

  }

  public void updateAutonCommand() {
    String ColorSelect = SideColor.getSelected();
    String AutoSelect;
    if (ColorSelect == "None") {
      AutoSelect = "";
    } else {
      AutoSelect = ColorSelect + "-" + Autos.getSelected();
    }
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
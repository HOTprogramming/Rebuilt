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
  SendableChooser<String> BlueAutos = new SendableChooser<>();
  SendableChooser<String> RedAutos = new SendableChooser<>();
  SendableChooser<String> SideColor = new SendableChooser<>();

  public RobotContainer() {
    //RobotController.setBrownoutVoltage(Constants.brownoutVoltage); // stops stuttering under high load when the battery is good.

    drivetrain = new Drive(new DriveKraken());

    

    BlueAutos.setDefaultOption("Auto", "Auto");
    BlueAutos.addOption("Test", "Test");
    BlueAutos.addOption("Blue-Left-Shoot&Climb-Ramp From Neutral", "Blue-Left-Shoot&Climb-Ramp From Neutral");
    BlueAutos.addOption("Blue-Right-Shoot&Climb-Ramp From Neutral", "Blue-Right-Shoot&Climb-Ramp From Neutral");  
    BlueAutos.addOption("Blue-Left-Shoot&Climb-Depot&Outpost","Blue-Left-Shoot&Climb-Depot&Outpost");
    BlueAutos.addOption("Blue-Right-Shoot&Climb-Outpost&Depot","Blue-Right-Shoot&Climb-Outpost&Depot");
    BlueAutos.addOption("Blue-Center-Shoot&Climb-Outpost", "Blue-Center-Shoot&Climb-Outpost");
    BlueAutos.addOption("Blue-Right-Scatter-Neutral", "Blue-Right-Scatter-Neutral");
    BlueAutos.addOption("Blue-Left-Scatter-Neutral", "Blue-Left-Scatter-Neutral");
    SmartDashboard.putData("Blue Autos",BlueAutos);
    
    RedAutos.setDefaultOption("Auto", "Auto");
    RedAutos.addOption("Test", "Test");
    RedAutos.addOption("Red-Left-Shoot&Climb-Ramp From Neutral", "Red-Left-Shoot&Climb-Ramp From Neutral");
    RedAutos.addOption("Red-Right-Shoot&Climb-Ramp From Neutral", "Red-Right-Shoot&Climb-Ramp From Neutral");
    RedAutos.addOption("Red-Left-Shoot&Climb-Depot&Outpost","Red-Left-Shoot&Climb-Depot&Outpost");
    RedAutos.addOption("Red-Right-Shoot&Climb-Outpost&Depot","Red-Right-Shoot&Climb-Outpost&Depot"); 
    RedAutos.addOption("Red-Center-Shoot&Climb-Outpost", "Red-Center-Shoot&Climb-Outpost");
    RedAutos.addOption("Red-Right-Scatter-Neutral", "Red-Right-Scatter-Neutral");
    RedAutos.addOption("Red-Left-Scatter-Neutral", "Red-Left-Scatter-Neutral");
    SmartDashboard.putData("Red Autos",RedAutos);

    SideColor.setDefaultOption("Red","Red");
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
    String BlueSelect = BlueAutos.getSelected();
    String RedSelect = RedAutos.getSelected();
    String ColorSelect = SideColor.getSelected();
    if (autoString != null) {
      if (ColorSelect == "Red") {
        if (RedSelect != autoString) {
        autoString = RedSelect;
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
        if (BlueSelect != autoString) {
        autoString = BlueSelect;
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
      }
    } else {
      if (ColorSelect == "Red") {
        autoString = RedSelect;
      } else {
        autoString = BlueSelect;
      }
    }
  }

  public Command getAutonomousCommand() {
    return autoCommand;
  }
}
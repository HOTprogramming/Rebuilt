// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
package frc.robot;

import java.io.IOException;


import org.json.simple.parser.ParseException;

import com.ctre.phoenix6.Utils;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.events.EventTrigger;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain.Drive;
import frc.robot.subsystems.Drivetrain.DriveSim;
import frc.robot.subsystems.Drivetrain.DriveKraken;



public class RobotContainer {

  private SendableChooser<String> chooser = new SendableChooser<>();
  private Command autoCommand;
  private String autoString;

  private Drive drivetrain;
  
  private final CommandXboxController driver = new CommandXboxController(0);
  private final CommandXboxController operator = new CommandXboxController(1);
  

  public RobotContainer() {
    //RobotController.setBrownoutVoltage(Constants.brownoutVoltage); // stops stuttering under high load when the battery is good.

    if(!Utils.isSimulation()){
        drivetrain = new Drive(new DriveKraken());
    } else {
        drivetrain = new Drive(new DriveSim());
    }

    chooser.setDefaultOption("Auto", "Auto");
    
    chooser.addOption("RedR4", "RedR4");
    chooser.addOption("RedL4", "RedL4");
    chooser.addOption("BlueR4", "BlueR4"); 
    chooser.addOption("BlueL4", "BlueL4"); 
    chooser.addOption("Funky Check", "Funky Check"); 
    chooser.addOption("RightBlueLolipop", "RightBlueLolipop"); 
    chooser.addOption("RightRedLolipopp", "RightRedLolipop"); 
    chooser.addOption("RedLeftLolipop", "RedLeftLolipop"); 
    chooser.addOption("BlueLeftLolipop", "BlueLeftLolipop"); 
    chooser.addOption("MiniBackshot", "MiniBackshot"); 
    chooser.addOption("MiniBackshot-optimized", "MiniBackshot-optimized"); 
    chooser.addOption("sacrifice", "Backshot-full"); 

    chooser.addOption("Copy of Regular4", "Copy of Regular4"); 
    chooser.addOption("Test of Regular4", "TestofRegular4"); 
    chooser.addOption("BLUE FLOOR TEST", "BLUE FLOOR TEST"); 
    chooser.addOption("BlueRLoliFetch", "BlueRLoliFetch"); 
    chooser.addOption("BlueLeftLoliFetch", "BlueLeftLoliFetch"); 

    // NamedCommands.registerCommand("Align Reef Left",  drivetrain.autonAlignReefCommand(0));
    // NamedCommands.registerCommand("Align Reef Center",  drivetrain.autonAlignReefCommand(1));
    // NamedCommands.registerCommand("Align Reef Right",  drivetrain.autonAlignReefCommand(2));
    // NamedCommands.registerCommand("Stop Drive", drivetrain.runOnce(() -> drivetrain.teleopDrive(0, 0, 0)));
    // NamedCommands.registerCommand("Auton Fetch 2M", drivetrain.fetchAuto(2.0, 1.0));
    // NamedCommands.registerCommand("Auton Fetch 15M", drivetrain.fetchAuto(1.2, 0.8));
    // NamedCommands.registerCommand("Chase Object", drivetrain.run(() -> drivetrain.chaseSlow()).until(() -> gamespecManager.intakeSubsystem.getBeamBreak()).onlyIf(() -> drivetrain.targetSeen)); // gamespecManager.intakeSubsystem.getBeamBreak() ||
    
    // NamedCommands.registerCommand("Chase Auton", drivetrain.run(() -> drivetrain.chaseAuton()).until(() -> gamespecManager.intakeSubsystem.getBeamBreak()).onlyIf(() -> drivetrain.targetSeen)); // gamespecManager.intakeSubsystem.getBeamBreak() ||
   // NamedCommands.registerCommand("Chase Object", drivetrain.run(() -> drivetrain.chaseSlow()).until(() -> (gamespecManager.intakeSubsystem.getBeamBreak()))); // gamespecManager.intakeSubsystem.getBeamBreak() ||

    // must make a runonce command using a functional command interface
    // NamedCommands.registerCommand("Auton Align Left",  drivetrain.run(() -> drivetrain.alignReef(0)));
    // NamedCommands.registerCommand("Auton Align Right", drivetrain.run(() -> drivetrain.alignReef(1)));


    SmartDashboard.putData(chooser);

    configureBindings();
  }

  private void configureBindings() {   

    //driver.a().onTrue(drivetrain.fetchAuto(2, 1.0)).onFalse(drivetrain.runOnce(() -> drivetrain.teleopDrive(0, 0, 0)));

    // drivetrain.setDefaultCommand
    //   (drivetrain.run(() -> {
    //         drivetrain.headingControl(
    //           Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY() : 0, 
    //           Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0);
    //       }
    //   ));    

      // driver.axisLessThan(0, -0.00)
      //   .or(driver.axisGreaterThan(0, 0.00))
      //   .or(driver.axisLessThan(1, -0.00))
      //   .or(driver.axisGreaterThan(1, 0.00))
      //   .and(driver.y().negate())
      //   .whileTrue

     /*  drivetrain.setDefaultCommand(
        drivetrain.run(()-> {
          double rawY = -driver.getLeftY();
          double rawX = -driver.getLeftX();
          double rawRot = -driver.getRightX();

         // double y = Math.abs(rawY) >=0.05 ? rawY : 0.0;
          //double x = Math.abs(rawX) >=0.05 ? rawX : 0.0;
          //double rot = Math.abs(rawRot) >=0.05 ? rawRot : 0.0;

          double rampedY = yLimiter.calculate(rawY);
          double rampedX = xLimiter.calculate(rawX);
          double rampedRot = rotLimiter.calculate(rawRot);

          drivetrain.teleopDrive(rampedY, rampedX, rampedRot);
        })
      );*/




        
     // );
 
      drivetrain.setDefaultCommand
      (drivetrain.run(() -> {
        drivetrain.teleopDrive(
          Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY() : 0,
          Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0,
          Math.abs(driver.getRightX()) >= 0.015 ? -driver.getRightX() : 0);
        }
      ));
      
      // .onFalse(Commands.race(Commands.waitSeconds(0.15), drivetrain.run(() -> {
      //   drivetrain.teleopDrive(
      //     Math.abs(driver.getLeftY()) >= 0.0 ? -driver.getLeftY() : 0,
      //     Math.abs(driver.getLeftX()) >= 0.0 ? -driver.getLeftX() : 0,
      //     Math.abs(driver.getRightX()) >= 0.015 ? -driver.getRightX() : 0);
      //   })));

      // driver.leftBumper().whileTrue(drivetrain.run(() -> {
      //   drivetrain.teleopDriveSlow(
      //     Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
      //     Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
      //     Math.abs(driver.getRightX()) >= 0.15 ? -driver.getRightX() : 0);
      // }))
      // .onFalse(Commands.race(Commands.waitSeconds(0.15), drivetrain.run(() -> {
      //   drivetrain.teleopDrive(
      //     Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
      //     Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
      //     Math.abs(driver.getRightX()) >= 0.15 ? -driver.getRightX() : 0);
      //   })));

      // driver.a().whileTrue(
      //   drivetrain.run(() -> {drivetrain.alignObjectTeleop(
      //     (Math.abs(driver.getLeftX()) >= 0.1 ? driver.getLeftX() : 0) * 0.5,
      //     (Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0) * 0.5,
      //     (Math.abs(driver.getRightX()) >= 0.015 ? -driver.getRightX() : 0) * 0.5);
      //   }
      // ));


      // driver.y()
      // .and(driver.axisLessThan(4, -0.15).or(driver.axisGreaterThan(4, 0.15))
      // .or(driver.axisLessThan(5, -0.15)).or(driver.axisGreaterThan(5, 0.15)))
      // .whileTrue
      // (drivetrain.run(() -> {
      //   drivetrain.lockReefManual(
      //     Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
      //     Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
      //     Math.abs(driver.getRightX()) >= 0.1 ? -driver.getRightX() : 0,
      //     Math.abs(driver.getRightY()) >= 0.1 ? -driver.getRightY() : 0);
      //   }
      // ));
      // b right y middle x left
      // driver.rightBumper().whileTrue(Commands.sequence(Commands.parallel(drivetrain.runOnce(() -> drivetrain.updateReefTarget(2)), gamespecManager.setLightsAligning()), drivetrain.resetControllers(), drivetrain.run(() -> drivetrain.alignReefRobotcentric(false)))).onFalse(refreshLights());
      // driver.y().whileTrue(Commands.sequence(Commands.parallel(drivetrain.runOnce(() -> drivetrain.updateReefTarget(1)), gamespecManager.setLightsAligning()), drivetrain.resetControllers(), drivetrain.run(() -> drivetrain.alignReefRobotcentric(false)))).onFalse(refreshLights());
      // driver.leftBumper().whileTrue(Commands.sequence(Commands.parallel(drivetrain.runOnce(() -> drivetrain.updateReefTarget(0)), gamespecManager.setLightsAligning()), drivetrain.resetControllers(), drivetrain.run(() -> drivetrain.alignReefRobotcentric(false)))).onFalse(refreshLights());

      // driver.rightBumper().whileTrue(drivetrain.run(() -> drivetrain.alignReefRobotcentric(false)));
      // driver.y().whileTrue(drivetrain.run(() -> drivetrain.alignReefRobotcentric(false)));
      // driver.leftBumper().whileTrue(drivetrain.run(() -> drivetrain.alignReefRobotcentric(false)));

      // driver.rightBumper().onFalse(refreshLights());
      // driver.y().onFalse(refreshLights());
      // driver.leftBumper().onFalse(refreshLights());

      // driver.b().whileTrue(NamedCommands.getCommand("Align Reef Left"));
      // driver.x().whileTrue(NamedCommands.getCommand("Align Reef Right"));
      // driver.b().onTrue(NamedCommands.getCommand("expel"));
  

      // driver.start().onTrue(drivetrain.resetPidgeon()).onFalse(drivetrain.run(() -> {
      //   drivetrain.teleopDrive(
      //     Math.abs(driver.getLeftY()) >= 0.1 ? -driver.getLeftY() : 0,
      //     Math.abs(driver.getLeftX()) >= 0.1 ? -driver.getLeftX() : 0,
      //     Math.abs(driver.getRightX()) >= 0.15 ? -driver.getRightX() : 0);
      //   }));

    // driver.a().whileTrue(Commands.runOnce(() -> turretSubsystem.gotoPosition(10.0)));
    // driver.b().onTrue(Commands.runOnce(() -> turretSubsystem.gotoPosition(10.0)));
    // driver.x().onTrue(Commands.runOnce(() -> turretSubsystem.gotoPosition(10.0)));
    // driver.y().whileTrue(Commands.runOnce(() -> turretSubsystem.sendMessage()));

     // Schedule Intake command on button press, cancelling on release.
     //driver.a().whileTrue(turretSubsystem.intakeCommand()).whileFalse(turretSubsystem.idleCommand());
     //driver.b().whileTrue(turretSubsystem.ejectCommand()).whileFalse(turretSubsystem.idleCommand());
     //driver.x().whileTrue(turretSubsystem.idleCommand());
   
    

      // NamedCommands.registerCommand("OTF", drivetrain.generateOnTheFly());
      // NamedCommands.registerCommand("R_OTF", drivetrain.runOnTheFly());

      // new EventTrigger("OTF").onTrue(Commands.runOnce(() -> drivetrain.generateOnTheFly()));

  }

  public void updateAutonCommand() {
    if (autoString != null) {
      if (chooser.getSelected() != autoString) {
        autoString = chooser.getSelected();
        autoCommand = new PathPlannerAuto(autoString);
        try {
          drivetrain.setAutonStartPose(PathPlannerAuto.getPathGroupFromAutoFile(autoString).get(0).getStartingDifferentialPose());
        } catch (IOException e) {
          System.err.println("No Auto File");
        } catch (IndexOutOfBoundsException e) {
          System.err.println("no auto Paths");
        }
      }
    } else {
      autoString = chooser.getSelected();
    }
  }

  public Command getAutonomousCommand() {
    return autoCommand;
  }
}
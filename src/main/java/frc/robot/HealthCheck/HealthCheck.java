package frc.robot.HealthCheck;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.StandardOpenOption;
import java.util.List;
import java.io.*;

import javax.print.PrintException;

import com.ctre.phoenix6.configs.MotionMagicConfigs;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Intake.IntakeConstants;
import frc.robot.subsystems.Intake.IntakeKraken;
import frc.robot.subsystems.Shooter.ShooterConstants;
import frc.robot.subsystems.Shooter.ShooterKraken;

public class HealthCheck {
	public final IntakeKraken m_IntakeKraken = IntakeKraken.getInstance();
	public final ShooterKraken m_ShooterKraken = ShooterKraken.getInstance();
	
	private String[] resultData; 
	private Path filePath = Path.of("/HealthCheck/report.html");
	private Path resultsfilePath = Path.of("results.html");
	private String fileContent;

	
	private double IntakeExtendPosition = 0.0;
	private double ShooterLeftTop = 0.0;
	private double ShooterLeftBottom = 0.0;
	private double ShooterRightTop = 0.0;


	public HealthCheck() {
		NetworkTableInstance inst = NetworkTableInstance.getDefault();
		NetworkTable table = inst.getTable("HOT/HealthCheck");

		readTemplate();
		writeStats();
	}

	public void update() {

	}

	private void readTemplate() {
		  // Reading the file (as a single string)
        try {
            fileContent = Files.readString(filePath);
            System.out.println("\nFile content (read as String):");
            System.out.println(fileContent);
        } catch (IOException e) {
            System.err.println("Error reading file: " + e.getMessage());
        }
    }



	private void writeStats() {
		// Writing to a file
        try {
            String content = "Hello, Java NIO!\nThis is a text file example.";
            Files.writeString(
                resultsfilePath,
                content,
                StandardOpenOption.CREATE,      // Create file if it doesn't exist
                StandardOpenOption.TRUNCATE_EXISTING // Overwrite if exists
            );
            System.out.println("File written successfully.");
        } catch (IOException e) {
            System.err.println("Error writing file: " + e.getMessage());
        }
	}



	private void validateIntakePosition() {
		IntakeExtendPosition = m_IntakeKraken.getExtendPosition();

	}

	private void validateShooterSpeed() {
		//Get Motor Velocities 
		ShooterLeftTop = m_ShooterKraken.m_krakenFXLeftTop.motor.getVelocity().getValueAsDouble();
		ShooterLeftBottom = m_ShooterKraken.m_krakenFXLeftBottom.motor.getVelocity().getValueAsDouble();
		ShooterRightTop = m_ShooterKraken.m_krakenFXRightTop.motor.getVelocity().getValueAsDouble();


	}

	private void DisplayHealthData() {
  		// let table = "<table class='table table-striped table-sm'><thead><tr>";
  		// table += "<th scope='col'>Test ID</th><th scope='col'>Motor ID</th><th scope='col'>Motor Name</th><th scope='col'>Avg Speed</th>";
  		// table += "<th scope='col'>Avg Supply</th><th scope='col'>Avg Stator</th></tr></thead><tbody>"; 
  
  		// //loop through network tables data and build test table
  		// for(let i=0;i<d.Control.count;i++) {
    	// 	table += "<tr><td>" + (parseInt(i) + 1) + "</td>";
    	// 	table += "<td>" + d.Tests[i].canid + "</td>";
    	// 	table +="<td>" + d.Tests[i].name +"</td>";
    	// 	table +="<td>" + d.Tests[i].speedavg.toFixed(2) + "</td>";
    	// 	table +="<td>" + d.Tests[i].supplyavg.toFixed(2)  + "</td>";
    	// 	table +="<td>" + d.Tests[i].statoravg.toFixed(2)  + "</td></tr>";
  		// }
  		// table += "</tbody></table>";
  		// //put tables data on html page 
  		// document.getElementById('hctable').outerHTML = table;
	}




	public Command runHealthCheck() {
		return Commands.sequence(
			checkIntakeExtend()
		);
	}



	private Command checkIntakeExtend() {
		return Commands.sequence(
			Commands.runOnce(() -> {System.out.println("Checking Intake Extend...");}),
			Commands.runOnce(() -> {
				m_IntakeKraken.m_ExtendKraken.motor.getConfigurator()
					.apply(new MotionMagicConfigs()
						.withMotionMagicAcceleration(64)
						.withMotionMagicCruiseVelocity(64));
			}),
			Commands.waitSeconds(0.02), 
			Commands.runOnce(() -> {
				m_IntakeKraken.runAgitatePosition(IntakeConstants.kDeployedPosition);
			}),
			Commands.waitSeconds(1.00), 
			Commands.runOnce(() -> {
				validateIntakePosition();
			})
		);
	}

	private Command checkShooter() {
		return Commands.sequence(
			Commands.runOnce(() -> {System.out.println("Checking Shooter...");}),
			Commands.runOnce(() -> {m_ShooterKraken.runVelocity(ShooterConstants.kShooterSpeed);}),
			Commands.waitSeconds(1.00), 
			Commands.runOnce(() -> {validateShooterSpeed();})
		);
	}


}
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.BufferedReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.net.NetworkInterface;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Enumeration;
import java.util.List;

import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.ctre.phoenix6.SignalLogger;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import org.littletonrobotics.junction.LoggedRobot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends LoggedRobot {
  private Command m_autonomousCommand;

  // private RobotContainer m_robotContainer;
  
  static { 
    String serialAddress = HALUtil.getSerialNumber();
    System.out.println("Serial Address Found: " + serialAddress);
    if(serialAddress == Constants.SerialAddressConstants.TEST_SERIAL_ADDRESS){
      Constants.isTestBot = true; 
    }
    else{
      Constants.isCompBot = false; 
      Constants.isTestBot = false; 
    }
    if(!Constants.isCompBot && !Constants.isTestBot){
      SmartDashboard.putString("Serial Address", serialAddress);
      SmartDashboard.putString("Test Serial Address", Constants.SerialAddressConstants.TEST_SERIAL_ADDRESS);
    }
    // if serial address doesn't work at comp
			// Constants.isCompBot = true;
  }

  static {
		List<byte[]> macAddresses;
		try {
			macAddresses = getMacAddresses();
		} catch (IOException e) {
			System.out.println("Mac Address attempt unsuccessful");
			System.out.println(e);
			macAddresses = List.of();
		}
    System.out.println("Mac Address found: " + macAddresses);

		for (byte[] macAddress : macAddresses) {
			// first check if we are comp
      
			if (Arrays.compare(Constants.MacAddressConstants.TEST_ADDRESS, macAddress) == 0){
				Constants.isTestBot = true; 
				break;
			}
			// if neither is true
			else {
				Constants.isCompBot = false;
				Constants.isTestBot = false;
				System.out.println("New Mac Address Discovered!");
			}
		}
                if (!Constants.isCompBot && !Constants.isTestBot) {
			// array
			String[] macAddressStrings = macAddresses.stream()
					.map(Robot::macToString)
					.toArray(String[]::new);

			SmartDashboard.putStringArray("MAC Addresses", macAddressStrings);
      for(String addresses : macAddressStrings){
        System.out.println(addresses);  
      }
			 //adds MAC addresses to the dashboard
			SmartDashboard.putString("Test MAC Address", macToString(Constants.MacAddressConstants.TEST_ADDRESS));
      // SmartDashboard.putString("Wood MAC Address");
      
			// if mac address doesn't work at comp
			// Constants.isCompBot = true;
		}
    SmartDashboard.putBoolean("Test Bot", Constants.isTestBot);
    SmartDashboard.putBoolean("Wood Bot", Constants.isWoodBot);
    SmartDashboard.putBoolean("Practice Bot", Constants.isPracticeBot);
    SmartDashboard.putBoolean("Comp Bot", Constants.isCompBot);

  }

    private static List<byte[]> getMacAddresses() throws IOException {
      List<byte[]> macAddresses = new ArrayList<>();
  
      Enumeration<NetworkInterface> networkInterfaces = NetworkInterface.getNetworkInterfaces();
      // connect to network
      NetworkInterface networkInterface;
      while (networkInterfaces.hasMoreElements()) {
        networkInterface = networkInterfaces.nextElement();
  
        byte[] address = networkInterface.getHardwareAddress();
        if (address == null) {
          continue;
        }
  
        macAddresses.add(address);
      }
      return macAddresses;
    }
     
    
    private static String macToString(byte[] address) {
      // changes string characters
      StringBuilder builder = new StringBuilder();
      for (int i = 0; i < address.length; i++) {
        if (i != 0) {
          builder.append(':');
        }
        builder.append(String.format("%02X", address[i]));
      }
      return builder.toString();
    }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    // String serialAddress = getRoboRIOSerialNumber();
    // System.out.println(getRoboRIOSerialNumber());
    // System.out.println("Robot Serial Address found:" + getRoboRIOSerialNumber());
    // SmartDashboard.putString("Test Serial Address", serialAddress);
    //  Logger.recordMetadata("ProjectName", "MyProject"); // Set a metadata value

    // if (isReal()) {
    //     Logger.addDataReceiver(new WPILOGWriter("/U")); // Log to a USB stick
    //     Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
    //     new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
    // } else {
    //     setUseTiming(false); // Run as fast as possible
    //     String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
    //     Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
    //     Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
    // }

    // // Logger.disableDeterministicTimestamps() // See "Deterministic Timestamps" in the "Understanding Data Flow" page
    // Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
    // SignalLogger.start();
    // // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // // autonomous chooser on the dashboard.
    // m_robotContainer = new RobotContainer();

  }

  public static String getRoboRIOSerialNumber() {
    try {
        // Command to get the serial number of the RoboRIO
        String command = "udevadm info --query=property --name=/dev/ttyACM0 | grep SERIAL";
        Process process = Runtime.getRuntime().exec(command);
        BufferedReader reader = new BufferedReader(new InputStreamReader(process.getInputStream()));

        String line;
        while ((line = reader.readLine()) != null) {
            if (line.contains("SERIAL")) {
                // Extracting the serial number
                return line.split("=")[1];
            }
        }
    } catch (Exception e) {
        e.printStackTrace();
    }
    return null; // Serial number could not be read
}


  


  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.
    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
    // m_robotContainer.onDisable();
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    // m_autonomousCommand.schedule();
    // }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}

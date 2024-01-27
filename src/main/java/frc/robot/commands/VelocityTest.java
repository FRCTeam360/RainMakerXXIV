// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CommandSwerveDrivetrain;


public class VelocityTest extends Command {
  private CommandSwerveDrivetrain drivetrain;
  private double kP = 0.0;
  private double kD = 0.0;
  private double kI = 0.0;
  private double kFF = 0.0;

  private double setPosition = 0.0;
  
  private Timer time = new Timer();

  private boolean isAtTarget;

  /** Creates a new VelocityTest. */
  public VelocityTest(CommandSwerveDrivetrain drivetrain) {
    this.drivetrain = drivetrain;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivetrain);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("p", kP);
    // SmartDashboard.putNumber("i", kI);
    // SmartDashboard.putNumber("d", kD);
    // SmartDashboard.putNumber("ff", kFF);
    // SmartDashboard.putNumber("Set Position", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    // double p = SmartDashboard.getNumber("p", 0.0);
    // double i = SmartDashboard.getNumber("i", 0.0);
    // double d = SmartDashboard.getNumber("d", 0.0);
    // double ff = SmartDashboard.getNumber("ff", 0.0);


    // if ((p != kP)) {
    //   drivetrain.pidController.setP(p);
    //   kP = p;
    // }
    // if ((i != kI)) {
    //   drivetrain.pidController.setI(i);
    //   kI = i;
    // }
    // if ((d != kD)) {
    //   drivetrain.pidController.setD(d);
    //   kD = d;
    // }




    double setPoint = SmartDashboard.getNumber("Set Position", 0.0);
    if (setPoint != setPosition) {
      System.out.println("updating setpoint: " + setPoint);
      time.reset();
      time.start();
      isAtTarget = false;
      drivetrain.setControl(new SwerveRequest.FieldCentric().withVelocityX(setPoint));
      setPosition = setPoint;
    }

    if(Math.abs(setPoint - drivetrain.getVelocity()) < .1) {
      time.stop();
      isAtTarget = true;
    } else {
      isAtTarget = false;
    }

    // if(drivetrain.pidController.atSetpoint()){
    //   time.stop();
    // }
    // processVariable = encoder.getPosition();

    SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Output", drivetrain.getVelocity());
    SmartDashboard.putNumber("Error", setPoint - drivetrain.getVelocity());
    SmartDashboard.putNumber("Time elapsed", time.get());
    SmartDashboard.putBoolean("At target", isAtTarget);
    //SmartDashboard.putNumber("Process Variable", processVariable);
    
  }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    time.stop();
    time.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

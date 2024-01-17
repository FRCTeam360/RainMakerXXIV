// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Linkage;

public class SetLinkage extends Command {
  private double kP = 0.0;
  private double kD = 0.0;
  private double kI = 0.0;
  private double kFF = 0.0;

  private double setPosition = 0.0;

  private Linkage linkage = Linkage.getInstance();

  /** Creates a new SetLinkage. */
  public SetLinkage() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(linkage);

    linkage.pidController.setP(kP);
    linkage.pidController.setD(kD);
    linkage.pidController.setI(kI);
    linkage.pidController.setFF(kFF);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    SmartDashboard.putNumber("p", kP);
    SmartDashboard.putNumber("i", kI);
    SmartDashboard.putNumber("d", kD);
    SmartDashboard.putNumber("ff", kFF);
    SmartDashboard.putNumber("Set Position", 0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double p = SmartDashboard.getNumber("p", 0);
    double i = SmartDashboard.getNumber("i", 0);
    double d = SmartDashboard.getNumber("d", 0);
    double ff = SmartDashboard.getNumber("ff", 0);

    
    if ((p != kP)) {
      linkage.pidController.setP(p);
      kP = p;
    }
    if ((i != kI)) {
      linkage.pidController.setI(i);
      kI = i;
    }
    if ((d != kD)) {
      linkage.pidController.setD(d);
      kD = d;
    }

    linkage.setFFWScaling(ff);
    if(ff != kFF){
      kFF = ff;
    }

    double setPoint = SmartDashboard.getNumber("Set Position", 5.0);
    if (setPoint != setPosition) {
      System.out.println("updating setpoint: " + setPoint);

      linkage.pidController.setReference(setPoint, CANSparkBase.ControlType.kPosition);
      setPosition = setPoint;
    }
    // processVariable = encoder.getPosition();

    // SmartDashboard.putNumber("SetPoint", setPoint);
    SmartDashboard.putNumber("Output", linkage.getSpeed());
    SmartDashboard.putNumber("Position", linkage.encoder.getPosition());
    SmartDashboard.putNumber("Error", setPoint - linkage.encoder.getPosition());
    //SmartDashboard.putNumber("Process Variable", processVariable);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

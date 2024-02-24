// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.ClimberIO;
import frc.robot.io.IntakeIO.IntakeIOInputs;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ClimberIOSparkMax implements ClimberIO {

    private CANSparkMax leftMotor = new CANSparkMax(Constants.CLIMBER_LEFT_ID, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(Constants.CLIMBER_RIGHT_ID, MotorType.kBrushless);

    private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightMotor.getEncoder();


    private final double POSITION_CONVERSION = (1.215 * Math.PI) / 15; //motor rotations -> (pulley diameter inches * pi) / (5 * 3 gearbox) -> inches
    private final double MINIMUM_HEGIHT = 0;

    private double kP = 0;
    private double kI = 0;
    private double kD = 0;
    private double kFF = 0;
    private double roll = 0;
    private double leftError = 0;
    private double rightError = 0;
    double p = SmartDashboard.getNumber("p", 0.0);
    double i = SmartDashboard.getNumber("i", 0.0);
    double d = SmartDashboard.getNumber("d", 0.0);
    double ff = SmartDashboard.getNumber("ff", 0.0);
    double goalHeight = SmartDashboard.getNumber("goal height", 85.0);

    /** Creates a new ClimberIOSparkMax. */
    public ClimberIOSparkMax() {//counterclocwise is positve roll follows unit circle
        leftMotor.restoreFactoryDefaults();
        rightMotor.restoreFactoryDefaults();

        leftMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setIdleMode(IdleMode.kBrake);

        leftMotor.setInverted(false);
        rightMotor.setInverted(true);
    }

    @Override
    public void runLeft(double speed) {
        leftMotor.set(speed);
    }

    @Override
    public void runRight(double speed) {
        rightMotor.set(speed);
    }

    @Override
    public void runBoth(double leftSpeed, double rightSpeed) {
        leftMotor.set(leftSpeed);
        rightMotor.set(rightSpeed);
    }

    @Override
    public void stop() {
        leftMotor.stopMotor();
        rightMotor.stopMotor();
    }

    @Override
    public void zeroBoth() {
        leftEncoder.setPosition(0);
        rightEncoder.setPosition(0);
    }


    @Override
    public void level() {
        double roll = 0.0;

        if (roll > 1.0) {
            runLeft(-0.3);
            runRight(0.3);
        } else if (roll < -1.0) {
            runLeft(0.3);
            runRight(-0.3);
        } else {
            stop();
        }
    }

    @Override
    public boolean leftAboveMinHeight() {
        double height = leftEncoder.getPosition();
        return height >= MINIMUM_HEGIHT ? true : false;
    }

    @Override
    public boolean rightAboveMinHeight() {
        double height = rightEncoder.getPosition();
        return height >= MINIMUM_HEGIHT ? true : false;
    }


    @Override
    public double getLeftPosition() {
        return leftEncoder.getPosition();
    }

    @Override
    public double getRightPosition() {
        return rightEncoder.getPosition();
    }

    @Override
    public double getRoll() {
        return 0.0;
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberSpeedLeft = leftMotor.get();
        inputs.climberSpeedRight = rightMotor.get();
        inputs.climberLeftAmps = leftMotor.getOutputCurrent();
        inputs.climberRightAmps = rightMotor.getOutputCurrent();
        inputs.climberLeftPosition = leftEncoder.getPosition();
        inputs.climberRightPosition = rightEncoder.getPosition();
        inputs.climberLeftVelocity = leftEncoder.getVelocity();
        inputs.climberRightVelocity = rightEncoder.getVelocity();
        inputs.climberLeftVoltage = leftMotor.getAppliedOutput();
        inputs.climberRightVoltage = rightMotor.getAppliedOutput();
    }

    @Override
    public void setLeftHeight(double goalHeight) {
        leftEncoder.setPosition(goalHeight);
    }

    @Override
    public void setRightHeight(double goalHeight) {
        rightEncoder.setPosition(goalHeight);
    }

    @Override
    public void updatePIDF(double kP, double kI, double kD, double kFF) {
        if ((p != kP)) {
            kP = p;
          }
      
          if ((i != kI)) {
            kI = i;
          }
      
          if ((d != kD)) {
            kD = d;
          }
      
          if (ff != kFF) {
            kFF = ff;
          }
    }
}

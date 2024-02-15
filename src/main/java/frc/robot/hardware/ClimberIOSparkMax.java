// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.generated.PracticebotConstants;
import frc.robot.io.ClimberIO;
import frc.robot.io.IntakeIO.IntakeIOInputs;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ClimberIOSparkMax implements ClimberIO {
    private CommandSwerveDrivetrain drivetrain = PracticebotConstants.DriveTrain;

    private CANSparkMax leftMotor = new CANSparkMax(Constants.CLIMBER_LEFT_ID, MotorType.kBrushless); 
    private CANSparkMax rightMotor = new CANSparkMax(Constants.CLIMBER_RIGHT_ID, MotorType.kBrushless);

    private Pigeon2 pigeon = drivetrain.getPigeon2();

    /** Creates a new ClimberIOSparkMax. */
    public ClimberIOSparkMax() {
        rightMotor.restoreFactoryDefaults();
        leftMotor.restoreFactoryDefaults();

        rightMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setIdleMode(IdleMode.kBrake);

        rightMotor.setInverted(true);
        leftMotor.setInverted(false);
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
    public void level() {
        double roll = pigeon.getRoll().getValueAsDouble();
        if (roll > 1.0) {
            runLeft(0.1);
        } else if (roll < -1.0) {
            runRight(0.1);
        } else {
            stop();
        }
    }

    @Override
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.speedLeft = leftMotor.get();
        inputs.speedRight = rightMotor.get();
    }
}

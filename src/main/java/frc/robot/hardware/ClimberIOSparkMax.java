// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.ClimberIO;
import frc.robot.io.IntakeIO.IntakeIOInputs;
import frc.robot.subsystems.CommandSwerveDrivetrain;

public class ClimberIOSparkMax implements ClimberIO {

    private CANSparkMax leftMotor = new CANSparkMax(Constants.CLIMBER_LEFT_ID, MotorType.kBrushless);
    private CANSparkMax rightMotor = new CANSparkMax(Constants.CLIMBER_RIGHT_ID, MotorType.kBrushless);

    public SparkPIDController leftPIDController = leftMotor.getPIDController();
    private SparkPIDController rightPIDController = rightMotor.getPIDController();

    private RelativeEncoder leftEncoder = leftMotor.getEncoder();
    private RelativeEncoder rightEncoder = rightMotor.getEncoder();


    private final double POSITION_CONVERSION = 1; //motor rotations -> (pulley diameter inches * pi) / (5 * 3 gearbox) -> inches
    private final double MINIMUM_HEGIHT = 0;

    private final float leftRetractLimit = -57;
    private final float leftExtensionLimit = 60;

    private final float rightRetractLimit = -57;
    private final float rightExtensionLimit = 60;

    private static class UnloadedConstants {
        static final double leftkP = 1.0;
        static final double leftkI = 0.0001;
        static final double leftkD = 0;

        static final double rightkP = 1.0;
        static final double rightkI = 0.0001;
        static final double rightkD = 0;
    }

    private static class LoadedConstants {
        static final double leftkP = 0;
        static final double leftkI = 0;
        static final double leftkD = 0;

        static final double rightkP = 0;
        static final double rightkI = 0;
        static final double rightkD = 0;
    }

    /** Creates a new ClimberIOSparkMax. */
    public ClimberIOSparkMax() {// counterclocwise is positve roll follows unit circle
        leftMotor.restoreFactoryDefaults();
        leftMotor.setIdleMode(IdleMode.kBrake);
        leftMotor.setInverted(true);

        rightMotor.restoreFactoryDefaults();
        rightMotor.setIdleMode(IdleMode.kBrake);
        rightMotor.setInverted(false);

        leftMotor.setSoftLimit(SoftLimitDirection.kForward, leftExtensionLimit);
        leftMotor.setSoftLimit(SoftLimitDirection.kReverse, leftRetractLimit);
        leftMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        leftMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        rightMotor.setSoftLimit(SoftLimitDirection.kForward, rightExtensionLimit);
        rightMotor.setSoftLimit(SoftLimitDirection.kReverse, rightRetractLimit);
        rightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
        rightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

        leftPIDController.setP(UnloadedConstants.leftkP, 0); // SLOT 0 IS UNLOADED
        leftPIDController.setI(UnloadedConstants.leftkI, 0);
        leftPIDController.setD(UnloadedConstants.leftkD, 0);

        rightPIDController.setP(UnloadedConstants.rightkP, 0);
        rightPIDController.setI(UnloadedConstants.rightkI, 0);
        rightPIDController.setD(UnloadedConstants.rightkD, 0);

        leftPIDController.setP(LoadedConstants.leftkP, 1); // SLOT 1 IS LOADED
        leftPIDController.setI(LoadedConstants.leftkI, 1);
        leftPIDController.setD(LoadedConstants.leftkD, 1);

        rightPIDController.setP(LoadedConstants.rightkP, 1);
        rightPIDController.setI(LoadedConstants.rightkI, 1);
        rightPIDController.setD(LoadedConstants.rightkD, 1);
    }

    @Override
    public void updatePIDF(double P, double I, double D, double F) {
        leftPIDController.setP(P, 1); // SLOT 0 IS EXTENSION
        leftPIDController.setI(I, 1);
        leftPIDController.setD(D, 1);
        leftPIDController.setFF(F,1);

        rightPIDController.setP(P, 1); // SLOT 0 IS EXTENSION
        rightPIDController.setI(I, 1);
        rightPIDController.setD(D, 1);
        rightPIDController.setFF(F,1);
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

    /**
     * @param height is in inches :D
     */
    @Override
    public void setLeftHeight(double height) { // height should be in inches
        height = height / POSITION_CONVERSION;
        leftPIDController.setReference(height, ControlType.kPosition);
    }

    /**
     * @param height is in inches :D
     */
    @Override
    public void setRightHeight(double height) {
        height = height / POSITION_CONVERSION;
        rightPIDController.setReference(height, ControlType.kPosition);
    }

    @Override
    public void zeroBoth() {
        leftEncoder.setPosition(0.0);
        rightEncoder.setPosition(0.0);
    }
    public void level() {
        // double roll = 0.0;

        // if (roll > 1.0) {
        //     runLeft(-0.3);
        //     runRight(0.3);
        // } else if (roll < -1.0) {
        //     runLeft(0.3);
        //     runRight(-0.3);
        // } else {
        //     stop();
        // }
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
    public void updateInputs(ClimberIOInputs inputs) {
        inputs.climberLeftStatorCurrent = leftMotor.getOutputCurrent();
        inputs.climberRightStatorCurrent = rightMotor.getOutputCurrent();
        inputs.climberLeftPosition = leftEncoder.getPosition();
        inputs.climberRightPosition = rightEncoder.getPosition();
        inputs.climberLeftVelocity = leftEncoder.getVelocity();
        inputs.climberRightVelocity = rightEncoder.getVelocity();
        inputs.climberLeftVoltage = leftMotor.getAppliedOutput() * leftMotor.getBusVoltage();
        inputs.climberRightVoltage = rightMotor.getAppliedOutput() * rightMotor.getBusVoltage();
    }
}

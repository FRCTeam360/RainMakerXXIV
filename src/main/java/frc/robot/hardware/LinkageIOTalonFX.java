// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.SoftwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.hardware.core.CoreTalonFX;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.LinkageIO;


public class LinkageIOTalonFX implements LinkageIO {
  /** Creates a new IntakeIOtalonFX. */
  private final TalonFX talonFX = new TalonFX(Constants.LINKAGE_ID, "Default Name");
  // private final RelativeEncoder encoder = talonFX.getEncoder();
  // private final SparkPIDController pidController = talonFX.getPIDController();

  private Slot0Configs slot0Configs = new Slot0Configs();
  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  // TODO test code

  private PositionVoltage positionVoltage = new PositionVoltage(0);
  DigitalInput zeroButton = new DigitalInput(Constants.LINKAGE_ZERO_BUTTON_PORT);
  DigitalInput brakeButton = new DigitalInput(Constants.LINKAGE_BRAKE_TOGGLE_BUTTON_PORT);

  public LinkageIOTalonFX() {
    final double GEAR_RATIO = 360.0 / 36.0; // TODO: FIX LMAOO!! was 6.0??
    final double kP = 0.01;
    final double kD = 0.0;
    final double kI = 0.0;
    final double kFF = 0.0;

    final double forwardLimit = 0.0; // TODO: make sure these are correct for prac bot
    final double reverseLimit = 0.0;

    talonFX.getConfigurator().apply(new TalonFXConfiguration());
    talonFX.setInverted(false);
    talonFX.setNeutralMode(NeutralModeValue.Brake);

    // need to add offset??? 43.0 rn

    // talonFX.getConfigurator().apply(new SoftwareLimitSwitchConfigs()
    //     .withForwardSoftLimitThreshold(forwardLimit)
    //     .withReverseSoftLimitThreshold(reverseLimit)
    //     .withForwardSoftLimitEnable(true)
    //     .withReverseSoftLimitEnable(true)); // TODO: dont enable robot past soft limits

    // translated into talonfx from sparkmax, probalby unnecessary
    // talonFX.getConfigurator().apply(new
    // ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(1.0));

    slot0Configs.kP = kP;
    slot0Configs.kI = kI;
    slot0Configs.kD = kD;
  }

  public boolean getZeroButton(){
    return this.zeroButton.get();
  }

  public boolean getBrakeButton(){
    return this.brakeButton.get();
  }

  @Override
  public void updateInputs(LinkageIOInputs inputs) {
    inputs.linkageAngle = talonFX.getPosition().getValueAsDouble();
    inputs.linkageVoltage = talonFX.getMotorVoltage().getValueAsDouble();
  }

  public void set(double speed) {
    System.out.println("hardware speed " + speed);
    talonFX.setControl(dutyCycleOut.withOutput(speed));
  }

  public void stopMotor() {
    talonFX.stopMotor();
  }

  public double getPosition() {
    return talonFX.getPosition().getValueAsDouble();
  }

  public double get() {
    return talonFX.get();
  }

  public void setFF(double ff) {
    slot0Configs.kV = ff;
  }

  public double getAppliedOutput() {
    return talonFX.getDutyCycle().getValueAsDouble();
  }

  public void setPosition(double angle) {
    talonFX.setPosition(angle);
  }

  @Override
  public void setReference(double setPoint) { //TODO: TEST???
    this.positionVoltage.Position = setPoint;

    talonFX.setControl(this.positionVoltage);
  }
}

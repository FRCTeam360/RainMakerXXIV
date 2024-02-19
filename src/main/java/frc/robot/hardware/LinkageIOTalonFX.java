// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.configs.ClosedLoopRampsConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
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

  private DutyCycleOut dutyCycleOut = new DutyCycleOut(0);
  private TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

  private PositionVoltage positionVoltage = new PositionVoltage(0);
  DigitalInput zeroButton = new DigitalInput(Constants.LINKAGE_ZERO_BUTTON_PORT);
  DigitalInput brakeButton = new DigitalInput(Constants.LINKAGE_BRAKE_TOGGLE_BUTTON_PORT);

  /*
   * DO NOT USE ON ENCODER BC MOTION MAGIC WAS TUNED IN NATIVE UNITS
   * ONLY FOR SHUFFLEBOARD CONTROL :D
   */
  private final double GEAR_RATIO = 360.0 / 60.0;

  public LinkageIOTalonFX() {
    final double kA = 0.0;
    final double kD = 0.0;
    final double kG = 0.0;
    final double kI = 0.0;
    final double kP = 2.5;
    final double kS = 0.25;
    final double kV = 0.0;

    final double motionMagicAcceleration = 400.0;
    final double motionMagicCruiseVelocity = 85.0;
    final double motionMagicCruiseJerk = 1750.0;

    final double forwardLimit = 28.0; // TODO: make sure these are correct for prac bot
    final double reverseLimit = 0.0; // 29.5

    talonFX.getConfigurator().apply(new TalonFXConfiguration());
    talonFX.setInverted(false);
    talonFX.setNeutralMode(NeutralModeValue.Brake);

    // need to add offset??? 43.0 rn

    // translated into talonfx from sparkmax, probalby unnecessary
    // talonFX.getConfigurator().apply(new
    // ClosedLoopRampsConfigs().withDutyCycleClosedLoopRampPeriod(1.0));

    Slot0Configs slot0Configs = talonFXConfiguration.Slot0;
    slot0Configs.kA = kA;
    slot0Configs.kD = kD;
    slot0Configs.kG = kG;
    slot0Configs.kI = kI;
    slot0Configs.kP = kP;
    slot0Configs.kS = kS;
    slot0Configs.kV = kV;

    MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = motionMagicCruiseVelocity;
    motionMagicConfigs.MotionMagicAcceleration = motionMagicAcceleration;
    motionMagicConfigs.MotionMagicJerk = motionMagicCruiseJerk;

    talonFXConfiguration.Voltage.PeakForwardVoltage = 12.0;
    talonFXConfiguration.Voltage.PeakReverseVoltage = 12.0;

    talonFXConfiguration.SoftwareLimitSwitch
        .withForwardSoftLimitThreshold(forwardLimit)
        .withReverseSoftLimitThreshold(reverseLimit)
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true);

    talonFX.getConfigurator().apply(talonFXConfiguration, 0.050);
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
    speed = speed / GEAR_RATIO;
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
    ///uhhh doesnt work rn will fix later - lauren
    //slot0Configs.kV = ff;
  }

  public double getAppliedOutput() {
    return talonFX.getDutyCycle().getValueAsDouble();
  }

  public void setPosition(double angle) {
    angle = angle / GEAR_RATIO;
    talonFX.setPosition(angle);
  }

  @Override
  public void setReference(double setPoint) { //TODO: TEST???
    setPoint = setPoint / GEAR_RATIO;

    this.positionVoltage.Position = setPoint;

    talonFX.setControl(this.positionVoltage);
  }
}

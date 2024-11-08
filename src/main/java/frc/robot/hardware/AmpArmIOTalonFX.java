// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.hardware;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfigurator;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.io.AmpArmIO;

public class AmpArmIOTalonFX implements AmpArmIO {

  private final TalonFX armMotor = new TalonFX(Constants.AMP_ARM_ID, Constants.CANIVORE_NAME);
  private final TalonFX wristMotor = new TalonFX(Constants.AMP_WRIST_ID, Constants.CANIVORE_NAME);

  private final DigitalInput intakeSensor = new DigitalInput(Constants.AMP_INTAKE_SENSOR_PORT);

  private final double PRACTICE_ARM_RATIO = 1.68; // degrees / motor rotation
  private final double COMP_ARM_RATIO = 360.0 / ((5.0 / 1.0) * (36.0 / 14.0) * (60.0 / 18.0));
  private final double WRIST_RATIO = 360.0/(4.0*4.0*3.0); // degrees / motor rotation
  // was 11.25


  private final double ARM_FORWARD_LIMIT = 120.0;
  private final double ARM_REVERSE_LIMIT = -78.0;

  private final DutyCycleEncoder absEncoder = new DutyCycleEncoder(Constants.AMP_ARM_ABS_ENCODER);
  private final double zeroOffset = 0.0; // needs to be retuned

  private final double practiceArmKP = 0.48;
  private final double practiceArmKD = 0.0;
  private final double practiceArmKF = 0.0;
  private final double practiceArmKI = 0.0;

  private final double compArmKP = 0.3;
  private final double compArmKD = 0.0;
  private final double compArmKF = 0.0;
  private final double compArmKI = 0.0;

  private final double wristKP = 0.6;
  private final double wristKI = 0.0;
  private final double wristKD = 0.0;
  private final double wristKF = 0.0;

  private DigitalInput zeroButton;
  private DigitalInput brakeButton;

  private boolean zeroPrev = false;
  private boolean brakePrev = false;

  private NeutralModeValue neutralMode = NeutralModeValue.Brake;

  /** Creates a new AmpArmIOTalonFX. */
  public AmpArmIOTalonFX(DigitalInput zeroButton, DigitalInput brakeButton) {
    this.zeroButton = zeroButton;
    this.brakeButton = brakeButton;

    armMotor.getConfigurator().apply(new TalonFXConfiguration());
    wristMotor.getConfigurator().apply(new TalonFXConfiguration());

    TalonFXConfiguration armConfig = new TalonFXConfiguration();

    armConfig.SoftwareLimitSwitch
        .withForwardSoftLimitThreshold(ARM_FORWARD_LIMIT)
        .withReverseSoftLimitThreshold(ARM_REVERSE_LIMIT)
        .withForwardSoftLimitEnable(true)
        .withReverseSoftLimitEnable(true);

    armConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // SAME AS SET INVERTED LOL

    TalonFXConfiguration wristConfig = new TalonFXConfiguration();
    wristConfig.Feedback.withSensorToMechanismRatio(1 / WRIST_RATIO);

    Slot0Configs wristSlot0 = wristConfig.Slot0;
    wristSlot0.kP = wristKP;

    Slot0Configs armSlot0 = armConfig.Slot0;

    armConfig.Feedback.withSensorToMechanismRatio(1 / COMP_ARM_RATIO);
    armSlot0.kP = compArmKP;

    // if (Constants.isCompBot()) {
    // armConfig.Feedback.withSensorToMechanismRatio(1 / COMP_ARM_RATIO);
    // armSlot0.kP = compArmKP;
    // } else {
    // armConfig.Feedback.withSensorToMechanismRatio(1 / PRACTICE_ARM_RATIO);
    // armSlot0.kP = practiceArmKP;
    // }

    armMotor.getConfigurator().apply(armConfig);
    wristMotor.getConfigurator().apply(wristConfig);

    armMotor.setNeutralMode(NeutralModeValue.Brake);

    wristMotor.setInverted(false);
    wristMotor.setNeutralMode(NeutralModeValue.Brake);

  }

  public void enableBrakeMode() {
    neutralMode = NeutralModeValue.Brake;
    armMotor.setNeutralMode(NeutralModeValue.Brake);
    wristMotor.setNeutralMode(NeutralModeValue.Brake);

  }

  @Override
  public boolean getIntakeSensor() {
    return intakeSensor.get();
  }

  public void disableBrakeMode() {
    neutralMode = NeutralModeValue.Coast;
    armMotor.setNeutralMode(NeutralModeValue.Coast);
    wristMotor.setNeutralMode(NeutralModeValue.Coast);

  }

  @Override
  public void resetArmWristPos() {
    armMotor.setPosition(-78.0);
    wristMotor.setPosition(75.0);
    // was 70.0
  }

  @Override
  public void runArm(double speed) {
    armMotor.set(speed);
  }

  @Override
  public void setArm(double angle) {
    PositionVoltage positionVoltage = new PositionVoltage(angle);
    armMotor.setControl(positionVoltage);
  }

  @Override
  public void setWrist(double angle) {
    PositionVoltage positionVoltage = new PositionVoltage(angle);
    wristMotor.setControl(positionVoltage);
  }

  @Override
  public void runWrist(double speed) {
    wristMotor.set(speed);
  }

  @Override
  public void stopArm() {
    armMotor.set(0.0);
  }

  @Override
  public void stopWrist() {
    wristMotor.set(0.0);
  }

  public void updateInputs(AmpArmIOInputs inputs) {
    inputs.armStatorCurrent = armMotor.getStatorCurrent().getValueAsDouble();
    inputs.wristStatorCurrent = wristMotor.getStatorCurrent().getValueAsDouble();
    inputs.armSupplyCurrent = armMotor.getSupplyCurrent().getValueAsDouble();
    inputs.wristSupplyCurrent = wristMotor.getSupplyCurrent().getValueAsDouble();
    inputs.armVoltage = armMotor.getMotorVoltage().getValueAsDouble();
    inputs.wristVoltage = wristMotor.getMotorVoltage().getValueAsDouble();
    inputs.armVelocity = armMotor.getVelocity().getValueAsDouble();
    inputs.wristVelocity = wristMotor.getVelocity().getValueAsDouble();
    inputs.armPosition = armMotor.getPosition().getValueAsDouble();
    inputs.wristPosition = wristMotor.getPosition().getValueAsDouble();
    inputs.ampIntakeSensor = this.getIntakeSensor();
    inputs.zeroButton = this.getRawZeroButton();
    inputs.brakeButton = this.getRawBrakeButton();
  }

  @Override
  public double getArmPosition() {
    return armMotor.getPosition().getValueAsDouble();
  }

  @Override
  public double getWristPosition() {
    return wristMotor.getPosition().getValueAsDouble();
  }

  @Override
  public void zeroWrist() {
    wristMotor.setPosition(0.0);
  }

  @Override
  public void zeroArm() {
    armMotor.setPosition(0.0);
  }

  @Override
  public void setWrist70() {
    wristMotor.setPosition(70.0);
  }

  @Override
  public void setArm78() {
    armMotor.setPosition(-78.0);
  }

  private boolean getRawZeroButton() {
    return !this.zeroButton.get();
  }

  public boolean getZeroButton() {
    boolean zeroCurr = getRawZeroButton();
    boolean risingEdge = zeroCurr && !zeroPrev;
    zeroPrev = zeroCurr;
    return risingEdge;
  }

  private boolean getRawBrakeButton() {
    return !this.brakeButton.get();
  }

  public boolean getBrakeButton() {
    boolean brakeCurr = getRawBrakeButton();
    boolean risingEdge = brakeCurr && !brakePrev;
    brakePrev = brakeCurr;
    return risingEdge;
  }

  public boolean isBrakeMode() {
    return neutralMode == NeutralModeValue.Brake;
  }
}

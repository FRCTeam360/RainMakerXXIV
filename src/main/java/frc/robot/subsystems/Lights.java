// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.FireAnimation;
import com.ctre.phoenix.led.LarsonAnimation;
import com.ctre.phoenix.led.SingleFadeAnimation;
import com.ctre.phoenix.led.TwinkleAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.ctre.phoenix.led.LarsonAnimation.BounceMode;


public class Lights extends SubsystemBase {
  /** Creates a new Lights. */
  private final CANdle lights = new CANdle(14);
  


  public Lights() {
    lights.configLEDType(LEDStripType.RGB);
  }
  
  public void setGreen(){
    lights.setLEDs(85,240,101);
  }
  public void setRed(){
    lights.setLEDs(239,36,36);
  }
  public void setBlue(){
    lights.setLEDs(0,0,0);
  }
  public void setOrange(){
    lights.setLEDs(0,0,0);
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

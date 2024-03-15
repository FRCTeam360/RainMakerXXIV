// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.TwinkleAnimation;

public class Lights extends SubsystemBase {
  private final CANdle candle = new CANdle(9);
  /** Creates a new Lights. */
  public Lights() {
    candle.configFactoryDefault();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
  public void setLED(int red, int green, int blue){
    candle.setLEDs(red, green, blue);
  }
  public void setLEDsIndex(int red, int green, int blue, int white, int startIdx, int count){
    candle.setLEDs(red, green, blue, white, startIdx, count);
  }
  //off
  public void isOff(){
    this.setLED(0, 0, 0);
  }
  //pink
  public void isInitialized(){
    this.setLED(255, 105, 180);
  }
  //neon green
  public void limelight(){
    this.setLED(57, 255, 20);
  }
  //orange
  public void hasNote(){
    this.setLED(255, 128, 0);
  }
  //green
  public void isZeroed(){
    this.setLED(0, 145, 0);
  }
  //red
  public void notZeroed(){
    this.setLED(255, 0, 0);
  }
  //yellow
  public void lowBattery(){
    this.setLED(255, 125, 0);
  }
  //blue twinkle
  public void isBlueAlliance(){
    candle.animate(new TwinkleAnimation(0, 40, 95));
  }
  //red twinkle
  public void isRedAlliance(){
    candle.animate(new TwinkleAnimation(255, 0, 0));
  }
}

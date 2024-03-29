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
    //lights.configLEDType(LEDStripType.GRB);
    lights.configBrightnessScalar(1);

  }
  public void setGreen(){
    lights.setLEDs(4,212,132,0,0,9);
  }
  public void setRed(){
    lights.setLEDs(255, 0, 0,0,0,9);
  }
  public void setBlue(){
    lights.setLEDs(0,0,255,0,0,9);
  }
  public void setOrange(){
    lights.setLEDs(250,40,0,0,0,9);
  }
  public void setAnimation(int r, int g, int b,String animationString){
    lights.clearAnimation(0);
    lights.setLEDs(r, g, b);
    
    if(animationString.equals("Fire")){
      lights.animate(new FireAnimation());
    }else if(animationString.equals("Larson")){
      lights.animate(new LarsonAnimation(r,g,b));
    }else if(animationString.equals("Twinkle")){
      lights.animate(new TwinkleAnimation(r,g,b));
    }else if(animationString.equals("Fade")){
      lights.animate(new SingleFadeAnimation(r,g,b));
    }else if(animationString.equals("ColorFlow")){
      lights.animate(new ColorFlowAnimation(r, g, b));
    }
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}

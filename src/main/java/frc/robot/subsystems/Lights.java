// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.RobotState;
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
  private DigitalInput zeroButton;
  private Linkage linkage;
  private Vision vision;
  
  public Lights(DigitalInput zeroButton,Linkage linkage, Vision vision) {
    //lights.configLEDType(LEDStripType.GRB);
    setFirstRed();
    lights.configBrightnessScalar(1);
    this.zeroButton = zeroButton;
    this.linkage = linkage;
    this.vision = vision;
  }
  public void isZero(){
   if (zeroButton.get()) {
      setFirstBlue();
   }
  }

  public void linkageRange(){
    if(linkage.getAngle() >= 120){
      setFirstGreen();
    }
  }

  public void isTargetInView(){
    if (vision.isTargetInView()) {
      setAllOrange();
    }
  }
  
  public void setFirstRed(){
    lights.setLEDs(255, 0, 0,0,0,1);
  }
  public void setFirstGreen(){
    lights.setLEDs(4,212,132,0,0,1);
  }
  public void setFirstBlue(){
    lights.setLEDs(0,0,255,0,0,1);
  }
  public void setAllGreen(){
    lights.setLEDs(4,212,132,0,0,20);
  }
  public void setAllRed(){
    lights.setLEDs(255, 0, 0,0,8,20);
  }
  public void setAllBlue(){
    lights.setLEDs(0,0,255,0,8,20);
  }
  public void setAllOrange(){
    lights.setLEDs(250,40,0,0,8,20);
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
    if(RobotState.isDisabled()){
     isZero();
     linkageRange();
     if (DriverStation.isDSAttached() && DriverStation.getAlliance().get() == Alliance.Blue) {
      setAllBlue();
    } else {
      setAllOrange();
    }
    }else{
      if(!RobotState.isAutonomous()){
        isTargetInView();
      }
      
      }
  }
}

 
// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.trobot5013lib.TrobotUtil;
import frc.robot.trobot5013lib.led.BlinkingPattern;
import frc.robot.trobot5013lib.led.ChasePattern;
import frc.robot.trobot5013lib.led.RainbowPattern;
import frc.robot.trobot5013lib.led.SolidColorPattern;
import frc.robot.trobot5013lib.led.TrobotAddressableLED;

public class StatusLED extends SubsystemBase {
   private TrobotAddressableLED m_AddressableLED = new TrobotAddressableLED(9, 39);

   private Limelight mFrontLimelight;
   private LauncherShoulder mShoulder;

   private boolean mAdjustingWithoutTarget = false;
   
  /** Creates a new StatusLED. */
  public StatusLED(Limelight frontLL, LauncherShoulder shoulder) {
    mFrontLimelight = frontLL;
    mShoulder = shoulder;
  }

  @Override
  public void periodic() {
    // check conditions and set patterns
    boolean isDisabled = DriverStation.isDisabled();
    double matchTime = DriverStation.getMatchTime();

    if(!isDisabled){
      if(mFrontLimelight.hasTarget() && mShoulder.atGoal() && TrobotUtil.withinTolerance(mFrontLimelight.getTy().getDouble(50), 0-LimelightConstants.GETSPEAKERSKEW(), 1.5)){
        m_AddressableLED.setPattern(new SolidColorPattern(Color.kGreen));
      }else if(mFrontLimelight.hasTarget()){
        m_AddressableLED.setPattern(new BlinkingPattern(Color.kGreen, 0.5));
      } else{
        m_AddressableLED.setPattern(new BlinkingPattern(Color.kBlue, 0.5));
      }

      //if(mAdjustingWithoutTarget){
      //  m_AddressableLED.setPattern(new SolidColorPattern(Color.kRed));
      //}

      if(mShoulder.getGoal() == LauncherConstants.AMP_ANGLE_RADANS){
        m_AddressableLED.setPattern(new BlinkingPattern(Color.kPurple, 0.5));
      }
      if(mShoulder.getGoal() == LauncherConstants.DUCK_RADIANS && mShoulder.atGoal()){
        m_AddressableLED.setPattern(new BlinkingPattern(Color.kPink, 0.5));
      }

      if(matchTime >= 115 && matchTime <= 118){
        m_AddressableLED.setPattern(new RainbowPattern());
      }
    }
    else{
      m_AddressableLED.setPattern(new ChasePattern(new Color[]{Color.kRed, Color.kOrange, Color.kYellow, Color.kGreen, Color.kBlue, Color.kPurple}, 1));
    }

    m_AddressableLED.update();
  }

  public void adjustingWithoutTarget (boolean truth){
    mAdjustingWithoutTarget = truth;
  }
}

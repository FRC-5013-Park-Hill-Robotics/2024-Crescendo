// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.trobot5013lib.TrobotUtil;
import frc.robot.trobot5013lib.led.BlinkingPattern;
import frc.robot.trobot5013lib.led.ChaosPattern;
import frc.robot.trobot5013lib.led.ChasePattern;
import frc.robot.trobot5013lib.led.RainbowPattern;
import frc.robot.trobot5013lib.led.SolidColorPattern;
import frc.robot.trobot5013lib.led.TrobotAddressableLED;

public class StatusLED extends SubsystemBase {
   private TrobotAddressableLED m_AddressableLED = new TrobotAddressableLED(9, 39);

   private Limelight mFrontLimelight;
   private LauncherShoulder mShoulder;
   private IntakeWrist mIntake;

   private Boolean mShuttling = false;
   private String mPattern = "none";
   
  /** Creates a new StatusLED. */
  public StatusLED(Limelight frontLL, LauncherShoulder shoulder, IntakeWrist wrist) {
    mFrontLimelight = frontLL;
    mShoulder = shoulder;
    mIntake = wrist;
  }

  @Override
  public void periodic() {
    // check conditions and set patterns
    boolean isDisabled = DriverStation.isDisabled();
    boolean isAutonomous = DriverStation.isAutonomous();
    double matchTime = DriverStation.getMatchTime();

    if(isAutonomous && !isDisabled){
      if(mPattern != "Auto Chaos"){
        m_AddressableLED.setPattern(new ChasePattern(new Color[]{Color.kFirstRed, Color.kFirstBlue}, 3));
      }
      mPattern = "Auto Chaos";
    }
    else if(!isDisabled){
      if(matchTime >= 110 && matchTime <= 113){
        if(mPattern != "Match Time"){
          m_AddressableLED.setPattern(new RainbowPattern());
        }
        mPattern = "Match Time";
      }
      else if(mShuttling == true){
        if(mPattern != "Shuttling"){
          m_AddressableLED.setPattern(new BlinkingPattern(Color.kDarkRed, 0.1));
        }
        mPattern = "Shuttling";
      } 
      else if(mIntake.getGoal() == LauncherConstants.AMP_ANGLE_RADANS){
        if(mPattern != "Amping"){
          m_AddressableLED.setPattern(new BlinkingPattern(Color.kPurple, 0.1));
        }
        mPattern = "Amping";
      } 
      else if(mShoulder.getGoal() == LauncherConstants.DUCK_RADIANS){
        if(mPattern != "Ducking"){
          m_AddressableLED.setPattern(new BlinkingPattern(Color.kDeepPink, 0.1));
        }
        mPattern = "Ducking";
      } 
      else if(mFrontLimelight.hasTarget() && mShoulder.atGoal() && mFrontLimelight.getVerticalAngleOfErrorDegrees() < 0){
        if(mPattern != "Ready to Shoot"){
          m_AddressableLED.setPattern(new SolidColorPattern(Color.kGreen));
        }
        mPattern = "Ready to Shoot";
      } 
      else if(mFrontLimelight.hasTarget()){
        if(mPattern != "Has Target"){
          m_AddressableLED.setPattern(new BlinkingPattern(Color.kGreen, 0.1));
        }
        mPattern = "Has Target";
      } 
      else{
        if(mPattern != "None"){
        m_AddressableLED.setPattern(new BlinkingPattern(Color.kBlue, 0.1));
        }
        mPattern = "None";
      }

      SmartDashboard.putString("LED Command", mPattern);
    }
    else{
      if(mPattern != "Disabled Rainbow"){
        m_AddressableLED.setPattern(new ChasePattern(new Color[]{Color.kDarkRed, Color.kOrangeRed, Color.kYellow, Color.kGreen, Color.kBlue, Color.kPurple}, 1));
      }
      mPattern = "Disabled Rainbow";
    }
  }

  public void setShuttling (boolean truth){
    mShuttling = truth;
  }

  public Command setShuttlingCommand (boolean truth){
    Command theCommand = this.runOnce(()->{
      setShuttling(truth);
    });
    return theCommand;
  }
}

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.ReverseLimitSourceValue;
import com.ctre.phoenix6.signals.ReverseLimitTypeValue;

import edu.wpi.first.wpilibj2.command.Command;

public class Climber extends SubsystemBase {
  private final TalonFX leftClimberMotor = new TalonFX(CANConstants.CLIMBER_LEFT_CAN_ID);
  private final TalonFX rightClimberMotor = new TalonFX(CANConstants.CLIMBER_RIGHT_CAN_ID);

  //private VelocityVoltage m_BottomVoltage = new VelocityVoltage(0);
  //private VelocityVoltage m_topVoltage = new VelocityVoltage(0);
    // TODO
    //Create TalonFX motors for 
    //Create PID controller using pigeon to help avoid side load guessing .48 for P that would produce 2.4 v differnce at 5 degrees DONE
    //Create Control Request for Motor of tpe VoltageOut DONE

  /** Creates a new Climber. */
  public Climber() {

    //TODO: change inversions to correct values

    //limit switch?
    /* 
    HardwareLimitSwitchConfigs limitSwitchConfigs = new HardwareLimitSwitchConfigs();
    leftClimberMotor.getForwardLimit();
		leftClimberMotor.getReverseLimit();

    leftClimberMotor.getForwardLimit();
		leftClimberMotor.getReverseLimit();

    */

    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    leftConfig.MotorOutput.withInverted(InvertedValue.Clockwise_Positive);
    leftConfig.Feedback.FeedbackRotorOffset = 0;
    //leftConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
    //leftConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    //leftConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    leftClimberMotor.getConfigurator().apply(leftConfig);
    leftClimberMotor.set(0);


    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    rightConfig.MotorOutput.withNeutralMode(NeutralModeValue.Brake);
    rightConfig.MotorOutput.withInverted(InvertedValue.CounterClockwise_Positive);
    rightConfig.Feedback.FeedbackRotorOffset = 0;
    //leftConfig.HardwareLimitSwitch.ReverseLimitEna
    //rightConfig.HardwareLimitSwitch.ReverseLimitEnable = true;
    //rightConfig.HardwareLimitSwitch.ReverseLimitSource = ReverseLimitSourceValue.LimitSwitchPin;
    //rightConfig.HardwareLimitSwitch.ReverseLimitType = ReverseLimitTypeValue.NormallyOpen;
    rightClimberMotor.getConfigurator().apply(rightConfig);
    rightClimberMotor.set(0);
    //m_leftVoltage.withSlot(0);


        //Clear motor configs - config facgtory default

        //set motor configs, 
            //PID slot 0, //not sure how to do this
            //inversion.
            //idle mode brake.
            //limit switch //not sure how to do this


  }

  public TalonFX getLeftMotor() {
    return this.leftClimberMotor;
  }

  public TalonFX getRightMotor() {
    return this.rightClimberMotor;
  }

  
  public void leftClimberMove(double stickInput){
    leftClimberMotor.set(stickInput);
  }

  public void rightClimberMove(double stickInput){
    rightClimberMotor.set(stickInput);
  }
  

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command climbLeftCommand(double stickInput){
    Command result = run(() -> leftClimberMove(stickInput));
    return result;

    //TODO make climb command
    //apply 9.6 v to the low motor and 9.6 minus abssolute value of the pid for the high motor. +   gravity feed forward
    //stop when one motor hits extend limit switch
  }

  public Command climbRightCommand(double stickInput){
    Command result = run(() -> rightClimberMove(stickInput));
    return result;

    //TODO make climb command
    //apply 9.6 v to the low motor and 9.6 minus abssolute value of the pid for the high motor. +   gravity feed forward
    //stop when one motor hits extend limit switch
  }


  public Command resetClimberCommand(){
        //TODO make climb command
    //apply - 3 v +  feed forward
    //stop when one motor hits extend limit switch (if we don't have lower limit when amperage spikes)
    return null;
  }
}

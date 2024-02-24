// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.CANConstants;
import frc.robot.constants.ClimberConstants;
import frc.robot.constants.ClimberConstants;

import com.ctre.phoenix6.configs.HardwareLimitSwitchConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;

public class Climber extends SubsystemBase {
  private final TalonFX leftClimberMotor = new TalonFX(CANConstants.CLIMBER_LEFT_CAN_ID);
  private final TalonFX rightClimberMotor = new TalonFX(CANConstants.CLIMBER_RIGHT_CAN_ID);
  private ProfiledPIDController climberController = new ProfiledPIDController(ClimberConstants.kP, ClimberConstants.kI, ClimberConstants.kD, null);
  private VoltageOut voltage = new VoltageOut(0, false, false, false, false);

  //private VelocityVoltage m_BottomVoltage = new VelocityVoltage(0);
  //private VelocityVoltage m_topVoltage = new VelocityVoltage(0);
    // TODO
    //Create TalonFX motors for 
    //Create PID controller using pigeon to help avoid side load guessing .48 for P that would produce 2.4 v differnce at 5 degrees DONE
    //Create Control Request for Motor of tpe VoltageOut DONE

  /** Creates a new Climber. */
  public Climber() {
    //factory default left leftClimberMotor.configFactoryDefault();
    //factory default right leftClimberMotor.configFactoryDefault();

    //TODO: Config right motor

    rightClimberMotor.getConfigurator().apply(new TalonFXConfiguration());

    //TODO: change inversions to correct values
    rightClimberMotor.setInverted(true);

    rightClimberMotor.setNeutralMode(NeutralModeValue.Brake);

    /* 
    HardwareLimitSwitchConfigs limitSwitchConfigs = new HardwareLimitSwitchConfigs();
    leftClimberMotor.getForwardLimit();
		leftClimberMotor.getReverseLimit();

    leftClimberMotor.getForwardLimit();
		leftClimberMotor.getReverseLimit();

    */
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    leftConfig.Slot0.kP = ClimberConstants.kP;
    leftConfig.Slot0.kI = ClimberConstants.kI;
    leftConfig.Slot0.kD = ClimberConstants.kD;
    leftConfig.Slot0.kS = ClimberConstants.kS;
    leftConfig.Slot0.kV = ClimberConstants.kV;
    leftConfig.Slot0.kA = ClimberConstants.kA;
    leftClimberMotor.setNeutralMode(NeutralModeValue.Brake);
    leftClimberMotor.set(0);
    leftClimberMotor.setInverted(true);
    leftClimberMotor.getConfigurator().apply(leftConfig);
    //m_leftVoltage.withSlot(0);


        //Clear motor configs - config facgtory default

        //set motor configs, 
            //PID slot 0, //not sure how to do this
            //inversion.
            //idle mode brake.
            //limit switch //not sure how to do this


  }

  public void extendUp(){

  }

  public void extendDown(){

  }



  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public Command getClimbComand(){
    //TODO make climb command
    //apply 9.6 v to the low motor and 9.6 minus abssolute value of the pid for the high motor. +   gravity feed forward
    //stop when one motor hits extend limit switch
    return null;
  }
  public Command reseteClimberComand(){
        //TODO make climb command
    //apply - 3 v +  feed forward
    //stop when one motor hits extend limit switch (if we don't have lower limit when amperage spikes)
    return null;
  }
}

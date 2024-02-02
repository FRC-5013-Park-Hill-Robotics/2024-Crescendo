// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.Command;

public class Climber extends SubsystemBase {
    // TODO Create 2 motor controllers (left and right) of type TalonFx using the climber constants for the id

    //Create PID controller using pigeon to help avoid side load
    //Create Elevator Feed Forward controller 
    //Create Control Request for Motor of tpe VoltageOut
  /** Creates a new Climber. */
  public Climber() {
          //Clear motor configs - config facgtory default
        //set motor configs, 
            //PID slot 0, 
            //inversion.
            //idle mode brake.
            //limit switch
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
    //apply 9.6 v to the low motor and 9.6 minus abssolute value of the pid for the high motor. +  feed forward
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

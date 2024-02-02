// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.LauncherShoulder;

/** Add your docs here. */
public class IntakeCommandFactory {
    private IntakeRollers m_rollers;
    private IntakeWrist m_wrist;
    private LauncherShoulder m_shoulder;

    public IntakeCommandFactory(IntakeRollers rollers, IntakeWrist wrist, LauncherShoulder shoulder) {
        m_rollers = rollers;
        m_wrist = wrist;
        m_shoulder = shoulder;
    }
    public Command deployCommand(){
        Command theCommand =  m_wrist.run(()->{
           m_wrist.deploy();
        });

        theCommand.addRequirements(m_shoulder);
        return theCommand;
    }

    public Command retractCommand(){  
        Command theCommand =  m_wrist.run(()->{
           m_wrist.retract();
        });

        return theCommand;
    }

    public Command startRollersCommand(){
        Command theCommand = m_rollers.run(()->{
            m_rollers.feedIn();
        });

        return theCommand;
    }


    public Command stopRollersCommand(){
        Command theCommand = m_rollers.run(()->{
            m_rollers.stop();;
        });

        return theCommand;
    }
    public Command intakeSequenceCommand(){
        return deployCommand()
            .alongWith(startRollersCommand())
            .until(m_rollers::hasGamePiece).andThen(stopRollersCommand().alongWith(retractCommand()));
    }

}

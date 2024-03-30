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

    public IntakeCommandFactory(RobotContainer robotContainer) {
        m_rollers = robotContainer.getIntakeRollers();
        m_wrist = robotContainer.getIntakeWrist();
        m_shoulder = robotContainer.getLauncherShoulder();
    }
    public Command deployAndStartIntakeCommand(){
        Command theCommand =  m_wrist.runOnce(m_wrist::deploy);

        Command takeInCommand = m_rollers.takeIn();
        return theCommand.alongWith(takeInCommand);
    }

    public Command retractAndStopIntakeCommand(){  
        Command retractCommand =  m_wrist.run(m_wrist::retract).until(m_wrist::atGoal);

        Command stopCommand = m_rollers.stopC();

        return stopCommand.alongWith(retractCommand);
    }

    public Command startRollersCommand(){
        Command theCommand = m_rollers.runOnce(()->{
            m_rollers.feedIn();
        });

        return theCommand;
    }


    public Command stopRollersCommand(){
        Command theCommand = m_rollers.runOnce(()->{
            m_rollers.stop();
        });

        return theCommand;
    }

    public Command intakeSequenceCommand(){
        return deployAndStartIntakeCommand()
            .until(m_rollers::hasGamePiece).andThen(retractAndStopIntakeCommand());
    }
}

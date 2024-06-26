// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.RobotContainer;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.LauncherRollers;
import frc.robot.subsystems.LauncherShoulder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.StatusLED;

/** Add your docs here. */
public class CommandFactory {
    private CommandSwerveDrivetrain m_drivetrain;
    private Limelight m_limelight_front;
    private Limelight m_limelight_back;
    private LauncherRollers m_launcher_rollers;
    private IntakeRollers m_intake_rollers;
    private IntakeWrist m_intake_wrist;
    private StatusLED m_LED;
    private LauncherShoulder m_shoulder;
    private Supplier<Integer> m_gamepiece_pipeline;
    private Supplier<Integer> m_speaker_pipeline;
    private Supplier<Double> m_speaker_skew;

    public CommandFactory(RobotContainer robotContainer) {
        m_drivetrain = robotContainer.getDrivetrain();
        m_limelight_front = robotContainer.getFrontLimelight();
        //m_limelight_back = robotContainer.getBackLimelight();
        m_launcher_rollers = robotContainer.getLauncherRollers();
        m_intake_rollers = robotContainer.getIntakeRollers();
        m_intake_wrist = robotContainer.getIntakeWrist();
        m_shoulder = robotContainer.getLauncherShoulder();
        m_LED = robotContainer.getLED();
        m_gamepiece_pipeline = robotContainer::gamepiecePipeline;
        m_speaker_pipeline = robotContainer::getSpeakerPipeline;
        m_speaker_skew = LimelightConstants::GETSPEAKERSKEW;
    }
    public Command alignToGamepieceCommand() {
        return new AllignOnLLTarget(m_drivetrain, m_limelight_back, m_gamepiece_pipeline, m_speaker_skew);
    }
    public Command duckCommand() {
        return m_shoulder.goToSetpointCommand(LauncherConstants.DUCK_RADIANS);
    }
    public Command alignAndAdjustToSpeakerCommand() {
        Command align = new AllignOnLLTarget(m_drivetrain, m_limelight_front, m_speaker_pipeline, m_speaker_skew);
        Command adjust = new AutoAdjustAngle(m_launcher_rollers, m_shoulder);
        return align.alongWith(adjust);
    }
    public Command intakeRollerOutCommand() {
        return m_intake_rollers.throwOutManual();
    }
    
    public Command presetAngleAdjust(double setpoint) {
        return m_shoulder.goToSetpointUntilCompleteCommand(setpoint);
    }

    public Command lowerSpeed() {
        return m_launcher_rollers.setSpeedCommand(20);
    }

    public Command resetSpeed() {
        return m_launcher_rollers.setSpeedCommand(50);
    }

}

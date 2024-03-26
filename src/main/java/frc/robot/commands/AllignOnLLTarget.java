// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.constants.ThetaGains;
import frc.robot.subsystems.Limelight;
import frc.robot.trobot5013lib.TrobotUtil;

public class AllignOnLLTarget extends Command {
  private Limelight m_Limelight;
  private CommandSwerveDrivetrain m_Drivetrain;
  private Supplier<Integer> m_pipeline;
  private PIDController thetaController = new PIDController(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD);
  private boolean targeting = false;
  private Supplier<Double> m_skew ; 
  public AllignOnLLTarget(CommandSwerveDrivetrain drivetrain, Limelight Limelight, Supplier<Integer> pipeline, Supplier<Double> skewDegrees) {
    addRequirements(drivetrain);
    m_Drivetrain = drivetrain;
    m_Limelight = Limelight;
    m_pipeline = pipeline;
    m_skew = skewDegrees;
  }

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Limelight.setPipeline(m_pipeline.get());
    targeting = false;
    thetaController.reset();
    thetaController.setTolerance(Math.toRadians(1.5));
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("AllignOnLLTarget running", true);
    double thetaOutput = 0;
    double xOutput = 0;
    double yOutput = 0;
		if (m_Limelight.hasTarget()){
			double vertical_angle = m_Limelight.getHorizontalAngleOfErrorDegrees();
			double horizontal_angle = -m_Limelight.getVerticalAngleOfErrorDegrees() ;
			double setpoint = Math.toRadians(horizontal_angle)+ m_Drivetrain.getPose().getRotation().getRadians() + Math.toRadians(m_skew.get());
      thetaController.setSetpoint(setpoint);
      targeting = true;
			if (!thetaController.atSetpoint() ){
				thetaOutput = thetaController.calculate(m_Drivetrain.getPose().getRotation().getRadians(), setpoint);
			} 
      SmartDashboard.putNumber("targeting error", horizontal_angle);
		} 
    else {
			System.out.println("NO TARGET");
		}
    m_Drivetrain.setControl(drive.withVelocityX(xOutput).withVelocityY(yOutput).withRotationalRate(thetaOutput));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("AllignOnLLTarget running", false);
    m_Drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return targeting && TrobotUtil.withinTolerance(m_Limelight.getVerticalAngleOfErrorDegrees(), 0,3);
  }
}

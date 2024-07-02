package frc.robot.commands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.CommandSwerveDrivetrain;
import frc.robot.constants.DrivetrainConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.constants.ThetaGains;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.Limelight;

public class GotoNote extends Command {
  private Limelight m_Limelight;
  private CommandSwerveDrivetrain m_Drivetrain;
  private Supplier<Integer> m_pipeline;
  private IntakeWrist m_Wrist;
  private PIDController thetaController = new PIDController(ThetaGains.kP, ThetaGains.kI, ThetaGains.kD);
  private boolean targeting = false;

  private final SwerveRequest.RobotCentric drive = new SwerveRequest.RobotCentric()
    .withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private Timer m_timer = new Timer();
  boolean aquiredTarget = false;
  boolean lostGamepiece = false;
  boolean finished = false;


  public GotoNote(CommandSwerveDrivetrain drivetrain, Limelight Limelight, Supplier<Integer> pipeline, IntakeWrist wrist) {
    addRequirements(drivetrain);
    m_Drivetrain = drivetrain;
    m_Limelight = Limelight;
    m_pipeline = pipeline;
    m_Wrist = wrist;
    lostGamepiece = false;
    finished = false;
  }



  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    m_Limelight.setPipeline(m_pipeline.get());
    targeting = false;
    thetaController.reset();
    thetaController.setTolerance(LimelightConstants.ALLIGNMENT_TOLLERANCE_RADIANS);
    aquiredTarget = false;
    finished = false;
    lostGamepiece = false;
    m_timer = new Timer();
    SmartDashboard.putNumber("AutoGP Timer", m_timer.get());
    SmartDashboard.putBoolean("AutoGP Timer Bool", m_timer.get() < 0.05);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putBoolean("DriveToLLTarget running", true);
    double throttle = 0.5;
    double thetaOutput = 0;
    double xOutput = 0;
    SmartDashboard.putNumber("AutoGP Timer", m_timer.get());
    SmartDashboard.putBoolean("AutoGP Timer Bool", m_timer.get() < 0.05);
		if (m_Limelight.hasTarget()){
      aquiredTarget = true;
			double vertical_angle = m_Limelight.getVerticalAngleOfErrorDegrees();
			double horizontal_angle = -m_Limelight.getHorizontalAngleOfErrorDegrees() ;
			double setpoint = Math.toRadians(horizontal_angle)+ m_Drivetrain.getPose().getRotation().getRadians();
      thetaController.setSetpoint(setpoint);
      targeting = true;
		
			thetaOutput = thetaController.calculate(m_Drivetrain.getPose().getRotation().getRadians(), setpoint);
      //xOutput = -m_throttle.get()*DrivetrainConstants.maxSpeedMetersPerSecond;
		  xOutput = -throttle*DrivetrainConstants.maxSpeedMetersPerSecond;
      SmartDashboard.putNumber("targeting error", horizontal_angle);
		} else {
      if (aquiredTarget){
        if (!lostGamepiece){
          m_timer.start();
          lostGamepiece = true;
        }
        thetaOutput = 0;
        if (lostGamepiece && m_timer.get() < 0.05){
         xOutput = -throttle*DrivetrainConstants.maxSpeedMetersPerSecond;
        } else {
          xOutput = 0;
          finished = true;
        }
      } else {
        xOutput = 0;
      }
    }
    
    
    m_Drivetrain.setControl(drive.withVelocityX(xOutput).withRotationalRate(thetaOutput));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SmartDashboard.putBoolean("DriveToLLTarget running", true);
    m_Drivetrain.setControl(drive.withVelocityX(0).withVelocityY(0).withRotationalRate(0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return finished || m_Wrist.hasGamePieceAndDown();
  }
}
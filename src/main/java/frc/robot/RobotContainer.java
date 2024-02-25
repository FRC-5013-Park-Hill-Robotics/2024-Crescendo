// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AllignOnLLTarget;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.GamepadDrive;
import frc.robot.constants.LauncherConstants;
import frc.robot.constants.LimelightConstants;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.IntakeRollers;
import frc.robot.subsystems.IntakeWrist;
import frc.robot.subsystems.LauncherRollers;
import frc.robot.subsystems.LauncherShoulder;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.StatusLED;

public class RobotContainer {
  private double MaxSpeed = 6; // 6 meters per second desired top speed
  private double MaxAngularRate = 1.5 * Math.PI; // 3/4 of a rotation per second max angular velocity

  /* Setting up bindings for necessary control of the swerve drive platform */
  private final CommandXboxController driverController = new CommandXboxController(0); 
  private final CommandXboxController operatorController = new CommandXboxController(1); 
  private final CommandSwerveDrivetrain drivetrain = TunerConstants.DriveTrain; // My drivetrain

  private Climber m_climber = new Climber(); // creates the climber instance variable

  private IntakeRollers m_intakeRollers = new IntakeRollers(); // creates the intake rollers instance variable
  private IntakeWrist m_intakeWrist = new IntakeWrist(m_intakeRollers); // creates the intake wrist instance variable

  private LauncherRollers m_launcherRollers = new LauncherRollers(); // creates the launcher rollers instance variable
  private LauncherShoulder m_launcherShoulder = new LauncherShoulder(); // creates the launcher shoulder variable

  private Limelight m_LimelightFront = new Limelight("limelight-front", true); // creates the limelight front instance
                                                                               // variable
  private Limelight m_LimelightBack = new Limelight("limelight-back", true); // creates the limelight back instance
                                                                             // variable

  private StatusLED m_statusLED = new StatusLED(); // creates the status led instance variable

  private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
      .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
      .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // I want field-centric
                                                               // driving in open loop
  private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
  private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
  private final Telemetry logger = new Telemetry(MaxSpeed);
  private static RobotContainer instance;

  public RobotContainer() {
    super();
    instance = this;
    configureBindings();
  }

  private void configureBindings() {
    drivetrain.setDefaultCommand(new GamepadDrive(drivetrain, driverController));
    m_climber.setDefaultCommand(new ClimbCommand(m_climber));

    // reset the field-centric heading on left bumper press
    driverController.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldRelative()));
    driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.zeroGyroscope()));

 
    driverController.povLeft().onTrue(m_launcherRollers.incrementSpeedCommand(-5));
    driverController.povRight().onTrue(m_launcherRollers.incrementSpeedCommand(5));
    driverController.povUp().onTrue(m_launcherShoulder.incrementAngleCommand(Math.toRadians(1)));
    driverController.povDown().onTrue(m_launcherShoulder.incrementAngleCommand(Math.toRadians(-1)));


    driverController.rightBumper().whileTrue(m_intakeWrist.intakeGamePiece().andThen(rumbleSequence()))
        .onFalse(m_intakeWrist.retractCommand().andThen(stopRumbleCommand()));
    driverController.leftBumper().onTrue(m_intakeWrist.intakeGamePieceManualCommand())
        .onFalse(m_intakeWrist.intakeGamePieceManualEndCommand());

    driverController.y().onTrue(m_intakeRollers.throwOut());
    
    driverController.b().onTrue(m_launcherRollers.startCommand());
    driverController.x().onTrue(m_launcherRollers.stopCommand());

    new Trigger(m_intakeRollers::hasGamePiece)
      .onTrue(m_LimelightFront.setPipelineCommand(this::getSpeakerPipeline))
      .onTrue(m_LimelightBack.setPipelineCommand(LimelightConstants.APRIL_TAG_TARGETING))
      .onFalse(m_LimelightFront.setPipelineCommand(LimelightConstants.APRIL_TAG_TARGETING))
      .onFalse(m_LimelightBack.setPipelineCommand(LimelightConstants.GAME_PIECE_RECOGNITION));
    driverController.a().whileTrue(new AllignOnLLTarget(drivetrain, m_LimelightFront, this::getSpeakerPipeline)).onFalse(m_LimelightFront.setPipelineCommand(LimelightConstants.APRIL_TAG_TARGETING));

    //driverController.a().whileTrue(new InstantCommand(() -> m_LimelightFront.setTrust(true)))
    //    .onFalse(new InstantCommand(() -> m_LimelightFront.setTrust(false)));
    // new
    // Trigger(m_intakeRollers::hasGamePiece).onTrue(m_launcherRollers.startCommand());

    operatorController.a().whileTrue(new AmpCommand(m_launcherShoulder, m_intakeRollers, m_intakeWrist));
    operatorController.b().whileTrue(m_launcherShoulder.goToSetpointCommand(LauncherConstants.DUCK_RADIANS));

    operatorController.leftStick().whileTrue(m_climber.climbLeftCommand(operatorController.getLeftY()));
    operatorController.rightStick().whileTrue(m_climber.climbRightCommand(operatorController.getRightY()));

 
    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  public static RobotContainer getInstance() {
    return instance;
  }

  public LauncherShoulder getLauncherShoulder() {
    return m_launcherShoulder;
  }

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }

  public Command startRumbleCommand (){
    Command rumbleCommand = new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 1));
    return rumbleCommand;
  }

  public Command stopRumbleCommand () {
    Command stopRumbleCommand = new InstantCommand(() -> driverController.getHID().setRumble(RumbleType.kBothRumble, 0));
    return stopRumbleCommand;
  }
  public Command rumbleSequence() {
    return startRumbleCommand().andThen(stopRumbleCommand());

  }

  public Limelight getFrontLimelight() {
    return m_LimelightFront;
  }

    public Limelight getBackLimelight() {
    return m_LimelightBack;
  }

    public CommandXboxController getOperatorController() {
      return operatorController;
    }

    public int getSpeakerPipeline(){
      int pipeline = (DriverStation.getAlliance().get() == Alliance.Red)?LimelightConstants.APRIL_TAG_RED_SPEAKER:LimelightConstants.APRIL_TAG_BLUE_SPEAKER;
      return pipeline;
    }

    public Command ampCommand(){
      return m_launcherShoulder.ampAngleCommand()
        .andThen(m_intakeWrist.ampCommand())
        .andThen(m_intakeRollers.ampOutCommand());
    }
}

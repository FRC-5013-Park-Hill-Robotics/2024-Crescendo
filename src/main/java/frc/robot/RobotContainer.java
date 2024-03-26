// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.nio.file.Path;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.AllignOnLLTarget;
import frc.robot.commands.AmpCommand;
import frc.robot.commands.AutoAdjustAngle;
import frc.robot.commands.CommandFactory;
import frc.robot.commands.DriveToLLTarget;
import frc.robot.commands.ClimbCommand;
import frc.robot.commands.GamepadDrive;
import frc.robot.commands.IntakeCommandFactory;
import frc.robot.constants.AutoConstants;
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
import frc.robot.trobot5013lib.led.RainbowPattern;
import frc.robot.trobot5013lib.led.SolidColorPattern;
import frc.robot.trobot5013lib.led.TrobotAddressableLED;
import frc.robot.trobot5013lib.led.TrobotAddressableLEDPattern;

public class RobotContainer {
  private SendableChooser<Command> autoChooser;

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

  private Limelight m_LimelightFront = new Limelight("limelight-front", false); // creates the limelight front instance
                                                                               // variable
  private Limelight m_LimelightBack = new Limelight("limelight-back", false); // creates the limelight back instance
                                                                             // variable

  private IntakeCommandFactory m_IntakeCommandFactory = new IntakeCommandFactory(this);
  private CommandFactory m_CommandFactory = new CommandFactory(this);

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
    configureAutonomousCommands();
    //m_LimelightBack.setPipelineObjectDecection();

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    //SmartDashboard.putStringArray("Auto List", AutoBuilder.getAllAutoNames().toArray(new String[0]));

    m_LimelightFront.setPipeline(getSpeakerPipeline());
    m_LimelightBack.setPipeline(LimelightConstants.GAME_PIECE_RECOGNITION);
  }

  private void configureBindings() {
    //Default Commands
    drivetrain.setDefaultCommand(new GamepadDrive(drivetrain, driverController, m_LimelightFront));
    m_climber.setDefaultCommand(new ClimbCommand(m_climber));

    //Driver Controls
    // reset the field-centric heading on left bumper press
    driverController.back().onTrue(drivetrain.runOnce(() -> drivetrain.zeroGyroscope()));


    driverController.rightBumper()
        .whileTrue(m_intakeWrist.intakeGamePiece().andThen(rumbleSequence(driverController, 0.5)))
        .onFalse(m_intakeWrist.retractCommand().andThen(stopRumbleCommand(driverController)));
    driverController.leftBumper()
        .onTrue(m_intakeWrist.intakeGamePieceManualCommand())
        .onFalse(m_intakeWrist.intakeGamePieceManualEndCommand());

    driverController.y().onTrue(m_intakeRollers.throwOutManual()).onFalse(m_intakeRollers.stopC());

    driverController.x()
        .whileTrue(new DriveToLLTarget(drivetrain, m_LimelightBack, this::gamepiecePipeline, driverController::getRightTriggerAxis));
    
    driverController.a()
        .whileTrue(m_CommandFactory.alignAndAdjustToSpeakerCommand());
       // .onFalse(m_LimelightFront.setPipelineCommand(LimelightConstants.APRIL_TAG_TARGETING));

        /*
        .whileTrue(new AllignOnLLTarget(drivetrain, m_LimelightFront, this::getSpeakerPipeline, this::getSpeakerSkew)
        .alongWith(new AutoAdjustAngle(m_launcherRollers, m_launcherShoulder)
        //.andThen(m_intakeRollers.throwOut())
        )
        */        

    //operator controls
    operatorController.a().whileTrue(new AmpCommand(m_launcherShoulder, m_intakeRollers, m_intakeWrist)).onFalse(m_intakeRollers.ampOutCommand().andThen(m_intakeWrist.retractCommand()));
    operatorController.b().whileTrue(m_launcherShoulder.goToSetpointCommandContinuous(LauncherConstants.DUCK_RADIANS));
    operatorController.x().whileTrue(m_launcherShoulder.goToSetpointCommandContinuous(LauncherConstants.SPEAKER_ANGLE_RADIANS));
    operatorController.y().whileTrue(m_launcherShoulder.goToSetpointCommandContinuous(LauncherConstants.PODIUM_ANGLE_RADIANS));

    operatorController.leftStick().whileTrue(m_climber.climbLeftCommand(operatorController.getLeftY()));
    operatorController.rightStick().whileTrue(m_climber.climbRightCommand(operatorController.getRightY()));

    operatorController.leftBumper().onTrue(m_launcherRollers.startCommand());
    operatorController.rightBumper().onTrue(m_launcherRollers.stopCommand());

    //Calibration controls for the launcher
    operatorController.povLeft().onTrue(m_launcherRollers.incrementSpeedCommand(-5));
    operatorController.povRight().onTrue(m_launcherRollers.incrementSpeedCommand(5));
    operatorController.povUp().onTrue(m_launcherShoulder.incrementAngleCommand(Math.toRadians(1)));
    operatorController.povDown().onTrue(m_launcherShoulder.incrementAngleCommand(Math.toRadians(-1)));

    operatorController.leftTrigger().whileTrue(m_launcherShoulder.goToSetpointCommandContinuous(LauncherConstants.PODIUM_ANGLE_RADIANS).alongWith(m_launcherRollers.setSpeedCommand(45))).onFalse(m_launcherRollers.setSpeedCommand(50));

    // operatorController.povLeft().whileTrue(drivetrain.runDriveQuasiTest(Direction.kForward));
    // operatorController.povRight().whileTrue(drivetrain.runDriveQuasiTest(Direction.kReverse));

    // operatorController.povUp().whileTrue(drivetrain.runDriveDynamTest(Direction.kForward));
    // operatorController.povDown().whileTrue(drivetrain.runDriveDynamTest(Direction.kReverse));

    //other events
    // new Trigger(m_intakeRollers::hasGamePiece)
    //     .onTrue(m_LimelightFront.setPipelineCommand(this::getSpeakerPipeline))
    //     //  .alongWith(m_LimelightBack.setPipelineCommand(LimelightConstants.APRIL_TAG_TARGETING)))
    //     .onFalse(m_LimelightFront.setPipelineCommand(LimelightConstants.APRIL_TAG_TARGETING));
    //    //   .alongWith(m_LimelightBack.setPipelineCommand(LimelightConstants.GAME_PIECE_RECOGNITION)));


    // driverController.a().whileTrue(new InstantCommand(() ->
    // m_LimelightFront.setTrust(true)))
    // .onFalse(new InstantCommand(() -> m_LimelightFront.setTrust(false)));
    // new
    // Trigger(m_intakeRollers::hasGamePiece).onTrue(m_launcherRollers.startCommand());



    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
    }
    drivetrain.registerTelemetry(logger::telemeterize);
  }

  public void configureAutonomousCommands() {
    WaitCommand wait5 = new WaitCommand(0.5);
    NamedCommands.registerCommand("Align to Gamepiece", m_CommandFactory.alignToGamepieceCommand());
    NamedCommands.registerCommand("Intake Sequence", m_IntakeCommandFactory.intakeSequenceCommand());
    NamedCommands.registerCommand("Duck", m_CommandFactory.duckCommand());
    NamedCommands.registerCommand("Align and Adjust to Speaker", m_CommandFactory.alignAndAdjustToSpeakerCommand());
    NamedCommands.registerCommand("Intake Roller Out", m_CommandFactory.intakeRollerOutCommand().andThen(new WaitCommand(0.15)));
    
    NamedCommands.registerCommand("Auto Drive to Note", new DriveToLLTarget(drivetrain, m_LimelightBack, this::gamepiecePipeline, AutoConstants::AUTODRIVETONOTESPEED).withTimeout(0.35));
    NamedCommands.registerCommand("Auto Align to Speaker", new AutoAdjustAngle(m_launcherRollers, m_launcherShoulder));

    NamedCommands.registerCommand("Adjust to Subwoofer", m_CommandFactory.presetAngleAdjust(AutoConstants.SUBWOOFER));
    NamedCommands.registerCommand("Adjust to Subwoofer Side", m_CommandFactory.presetAngleAdjust(AutoConstants.SUBWOOFER_SIDE));
    NamedCommands.registerCommand("CloseAllign 2", m_CommandFactory.presetAngleAdjust(AutoConstants.TWO));
    NamedCommands.registerCommand("CloseAlign", m_CommandFactory.presetAngleAdjust(AutoConstants.TWO));
    NamedCommands.registerCommand("Shooter Allign 1", m_CommandFactory.presetAngleAdjust(AutoConstants.ONE));
    NamedCommands.registerCommand("Shooter Allign 2", m_CommandFactory.presetAngleAdjust(AutoConstants.TWO));
    NamedCommands.registerCommand("Shooter Allign 3", m_CommandFactory.presetAngleAdjust(AutoConstants.THREE));
    NamedCommands.registerCommand("Duck", m_CommandFactory.presetAngleAdjust(AutoConstants.DUCK));
    NamedCommands.registerCommand("Duck2", m_CommandFactory.presetAngleAdjust(AutoConstants.DUCK2));

    NamedCommands.registerCommand("Close To Wing Align", m_CommandFactory.presetAngleAdjust(AutoConstants.CLOSETOWING));

    NamedCommands.registerCommand("Intake Down", m_IntakeCommandFactory.deployAndStartIntakeCommand());
    NamedCommands.registerCommand("Intake Up", m_IntakeCommandFactory.retractAndStopIntakeCommand());

    NamedCommands.registerCommand("Lower Speed", m_CommandFactory.lowerSpeed());
    NamedCommands.registerCommand("Reset Speed", m_CommandFactory.resetSpeed());
    //NamedCommands.registerCommand("Intake Up", m_intakeWrist.intakeGamePieceManualEndCommand().until(m_intakeWrist::atGoal));
  }

  public Command getAutonomousCommand() {
    /* First put the drivetrain into auto run mode, then run the auto */
    return autoChooser.getSelected();
  }

  public PathPlannerAuto getPathPlannerAuto() {
    return (PathPlannerAuto) autoChooser.getSelected();
  }

  public static RobotContainer getInstance() {
    return instance;
  }

  public void setRumble(CommandXboxController controller, boolean rumble){
     controller.getHID().setRumble(RumbleType.kBothRumble, rumble?1:0);
  }

  public Command startRumbleCommand(CommandXboxController controller) {
    Command rumbleCommand = new InstantCommand(() -> setRumble(controller, true));
    return rumbleCommand;
  }

  public Command stopRumbleCommand(CommandXboxController controller) {
    Command rumbleCommand = new InstantCommand(() -> setRumble(controller, false));
    return rumbleCommand;
  }

  public Command rumbleSequence(CommandXboxController controller, double timeout) {
    return startRumbleCommand(controller).withTimeout(timeout).andThen(stopRumbleCommand(controller));
  }

  //------------------------------
  //  Return Fuctions
  //------------------------------
  public Limelight getFrontLimelight() {
    return m_LimelightFront;
  }

  /*
  public Limelight getBackLimelight() {
    return m_LimelightBack;
  }
   */

  public CommandSwerveDrivetrain getDrivetrain() {
    return drivetrain;
  }

  public IntakeWrist getIntakeWrist(){
    return m_intakeWrist;
  }

  public IntakeRollers getIntakeRollers(){
    return m_intakeRollers;
  }

  public LauncherRollers getLauncherRollers(){
    return m_launcherRollers;
  }

  public LauncherShoulder getLauncherShoulder(){
    return m_launcherShoulder;
  }

  public CommandXboxController getOperatorController() {
    return operatorController;
  }

  //------------------------------
  //  Math Return Fuctions
  //------------------------------
  public int getSpeakerPipeline() {
    int pipeline = (DriverStation.getAlliance().get() == Alliance.Red) ? LimelightConstants.APRIL_TAG_RED_SPEAKER
        : LimelightConstants.APRIL_TAG_BLUE_SPEAKER;
    return pipeline;
  }

  public Double getSpeakerSkew(){
    Alliance alliance = DriverStation.getAlliance().get();
    Double skew = -1.5;
    if (alliance == Alliance.Red) {
      return skew;
    }
    else {
      return -skew;
    }
  }

  public Double getGamepieceSkew(){
    Double skew = 0.0;
    return skew;
  }

  public int gamepiecePipeline(){
    return LimelightConstants.GAME_PIECE_RECOGNITION;
  }

  //------------------------------
  //  Commands for Robot
  //------------------------------
  public Command ampCommand() {
    Command movementCommand =  m_launcherShoulder.ampAngleCommand().alongWith(m_intakeWrist.ampCommand());
    Command ejectCommand = m_intakeRollers.ampOutCommand();
    return movementCommand.andThen(ejectCommand);
  }

}

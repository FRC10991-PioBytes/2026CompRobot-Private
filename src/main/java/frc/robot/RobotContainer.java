// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.OIConstants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveLEDSubsystem;
import frc.robot.subsystems.SwerveLEDSubsystem.LEDState;
import frc.robot.commands.AutomaticShootingCommands.FaceToPassCommand;
import frc.robot.commands.AutomaticShootingCommands.MoveToScorePosCommand;
import frc.robot.commands.AutomaticShootingCommands.ShootFromScorePosCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommands.RunIntakeInCommand;
import frc.robot.commands.IntakeCommands.RunIntakeOutCommand;
import frc.robot.commands.IntakeCommands.RunIntakePivotCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

/*
 * This class is where the bulk of the robot should be declared.  Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls).  Instead, the structure of the robot
 * (including subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer 
{
  // Subsystems
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intake = new IntakeSubsystem();
  private final ShooterSubsystem m_shooter = new ShooterSubsystem();
  private final FeederSubsystem m_feeder = new FeederSubsystem();
  //private final SwerveLEDSubsystem m_LEDs = new SwerveLEDSubsystem(m_robotDrive);

  // Driver controller
  CommandJoystick m_driverController = new CommandJoystick(OIConstants.kDriverControllerPort);
  
  // Operator controller
  CommandJoystick m_manipulatorController = new CommandJoystick(OIConstants.kManipulatorControllerPort);

  // Auto chooser
  private final SendableChooser<Command> pathAutoChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() 
  {
    DriverStation.silenceJoystickConnectionWarning(true);

    NamedCommands.registerCommand("StopFeeder", m_feeder.runOnce(() -> m_feeder.stop()));
    NamedCommands.registerCommand("StopShooter", m_shooter.runOnce(() -> m_shooter.stop()));
    NamedCommands.registerCommand("ExtendIntake", Commands.print("Extended Intake"));
    NamedCommands.registerCommand("RevShooter", m_shooter.runOnce(() -> m_shooter.setVelocity(4200)));
    NamedCommands.registerCommand("RunFeederAndShoot", m_feeder.runOnce(() -> m_feeder.setVelocity(4800)));
    NamedCommands.registerCommand("RunIntakeRoller", Commands.print("Running intake roller"));

    pathAutoChooser = AutoBuilder.buildAutoChooser("CenterStart-Score");

    SmartDashboard.putData("Auto/PP Autos", pathAutoChooser);

    //initCamera();

    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link edu.wpi.first.wpilibj.GenericHID} or one of its
   * subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then calling
   * passing it to a
   * {@link JoystickButton}.
   */
  private void configureButtonBindings() {
    
    //m_LEDs.setDefaultCommand(new RunCommand(() -> m_LEDs.setState(LEDState.Loading), m_LEDs));

    m_robotDrive.setDefaultCommand(new DriveCommand(m_robotDrive, //m_LEDs,
        () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.leftStickY), OIConstants.kDriveDeadband) * 0.714,
        () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.leftStickX), OIConstants.kDriveDeadband) * 0.714,
        () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.rightStickX), OIConstants.kDriveDeadband) * 0.57,
        () -> true));

    m_driverController.button(OIConstants.bumperLeft)
        .whileTrue(new DriveCommand(m_robotDrive, //m_LEDs,
            () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.leftStickY), OIConstants.kDriveDeadband) * 0.29,
            () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.leftStickX), OIConstants.kDriveDeadband) * 0.29,
            () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.rightStickX), OIConstants.kDriveDeadband) * 0.29,
            () -> true));
    
    m_driverController.button(OIConstants.buttonY)
        .whileTrue(new FaceToPassCommand(m_robotDrive,
        () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.leftStickY), OIConstants.kDriveDeadband),
        () -> -MathUtil.applyDeadband(m_driverController.getRawAxis(OIConstants.leftStickY), OIConstants.kDriveDeadband)));

    m_driverController.button(OIConstants.buttonB).and(m_driverController.button(OIConstants.bumperLeft)).and(m_driverController.button(OIConstants.bumperRight))
        .onTrue(m_robotDrive.runOnce(() -> m_robotDrive.zeroHeading()));
    
    // Move to closest scoring position
    m_driverController.button(OIConstants.buttonA)
        .whileTrue(new MoveToScorePosCommand(m_robotDrive));

    // Set X formation
    m_driverController.button(OIConstants.buttonX)
        .whileTrue(new RunCommand(
            () -> m_robotDrive.setX(),
            m_robotDrive));
    
    // Run shooter
    m_manipulatorController.axisGreaterThan(OIConstants.rightTrigger, 0.5)
        .onTrue(new ShootFromScorePosCommand(m_shooter, m_robotDrive))
        .onFalse(m_shooter.runOnce(() -> m_shooter.setVelocity(ShooterConstants.kShooterIdleVelocity)));
    
    // Run feeder
    m_manipulatorController.axisGreaterThan(OIConstants.leftTrigger, 0.5)
        .onTrue(m_feeder.runOnce(() -> m_feeder.setVelocity(4800)))
        .onFalse(m_feeder.runOnce(() -> m_feeder.stop()));

    // Pass
    m_manipulatorController.button(OIConstants.bumperRight)
        .whileTrue(m_shooter.runOnce(() -> m_shooter.setVelocity(5600)))
        .onFalse(m_shooter.runOnce(() -> m_shooter.setVelocity(ShooterConstants.kShooterIdleVelocity)));

    // Run intake
    m_manipulatorController.button(OIConstants.buttonX)
        .toggleOnTrue(new RunIntakeInCommand(m_intake));

    m_manipulatorController.button(OIConstants.buttonY)
        .toggleOnTrue(new RunIntakeOutCommand(m_intake));

    // Run agitator
    /*
    m_manipulatorController.button(OIConstants.buttonA)
        .whileTrue(m_feeder.runOnce(() -> m_feeder.runAgitator(-1)));
    */

    // Run intake
    m_intake.setDefaultCommand(new RunIntakePivotCommand(
        m_intake,
        () -> MathUtil.applyDeadband(m_manipulatorController.getRawAxis(OIConstants.rightStickY), 0.1)));
    

    // Sys Id Routines
    /*
    m_driverController.button(OIConstants.buttonA)
        .whileTrue(m_feeder.sysIdQuasistatic(SysIdRoutine.Direction.kForward));

    m_driverController.button(OIConstants.buttonB)
        .whileTrue(m_feeder.sysIdDynamic(SysIdRoutine.Direction.kForward));

    m_driverController.button(OIConstants.buttonX)
        .whileTrue(m_feeder.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

    m_driverController.button(OIConstants.buttonY)
        .whileTrue(m_feeder.sysIdDynamic(SysIdRoutine.Direction.kReverse));
    */
  }

  public Command getStopShootingCommand() {
    return Commands.parallel(m_shooter.runOnce(() -> m_shooter.stop()), m_feeder.runOnce(() -> m_feeder.stop()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    
    return pathAutoChooser.getSelected();
  }

  /*
  private void initCamera() {
    try {
        UsbCamera camera = CameraServer.startAutomaticCapture("MainCam", 0);
        camera.setResolution(320, 240);
        camera.setFPS(15);
    }
    catch (Exception e) {
        e.printStackTrace();
    }
  }
  */

}
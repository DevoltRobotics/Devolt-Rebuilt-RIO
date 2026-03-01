// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Commands.SOTFCommand;
import frc.robot.Constants.CANId;
import frc.robot.Constants.TurretsPos;
import frc.robot.Constants.CANId.CAN_Shooter;
import frc.robot.Subsystems.CommandSwerveDrivetrain;
import frc.robot.Subsystems.IntakeSubsystem;
import frc.robot.Subsystems.ShooterSubsystem;
import frc.robot.Subsystems.TransferSubsystem;
import frc.robot.Subsystems.TurretCoordinator;
import frc.robot.Subsystems.TurretSubsystem;

public class RobotContainer {
    private final SendableChooser<Command> autoChooser;

    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(2).in(RadiansPerSecond); // 3/4 of a rotation per second max
                                                                                   // angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();


    private final Telemetry logger = new Telemetry(MaxSpeed);

    public final CommandXboxController joystick = new CommandXboxController(0);
    public final Joystick buttonBoard = new Joystick(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    public final IntakeSubsystem intakeSubsystem = new IntakeSubsystem();

    public final TransferSubsystem transferSubsystem = new TransferSubsystem();

    public final TurretCoordinator turretCoordinator = new TurretCoordinator();

    private SparkMaxConfig turretLeftConfig = new SparkMaxConfig();
    public final SparkMax turretLeftMotor = new SparkMax(CANId.CAN_Shooter.LTurretCan, MotorType.kBrushless);

    public final ShooterSubsystem shooterLeftSubsystem = new ShooterSubsystem(CAN_Shooter.LFlywheelCan,
            drivetrain.getState().Pose);
    public final TurretSubsystem turretLeftSubsystem = new TurretSubsystem(
            turretLeftMotor,
            turretCoordinator,
            TurretCoordinator.Side.RIGHT,
            TurretsPos.LeftTurretOffset);

    private SparkMaxConfig turretRightConfig = new SparkMaxConfig();
    public final SparkMax turretRightMotor = new SparkMax(CANId.CAN_Shooter.RTurretCan, MotorType.kBrushless);

    public final ShooterSubsystem shooterRightSubsystem = new ShooterSubsystem(CAN_Shooter.RFlywheelCan,
            drivetrain.getState().Pose);
    public final TurretSubsystem turretRightSubsystem = new TurretSubsystem(
            turretRightMotor,
            turretCoordinator,
            TurretCoordinator.Side.LEFT,
            TurretsPos.RightTurretOffset);

         

    public RobotContainer() {

        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        NamedCommands.registerCommand("grab", new ParallelDeadlineGroup(new WaitCommand(3), intakeSubsystem.pivotDownCMD(), intakeSubsystem.IntakeinCMD(intakeSubsystem)));
        NamedCommands.registerCommand("intakeup", intakeSubsystem.pivotUpCMD());
        NamedCommands.registerCommand("intakedown", intakeSubsystem.pivotDownSafeCMD());
        NamedCommands.registerCommand("shoot", new SOTFCommand(drivetrain, shooterLeftSubsystem, turretLeftSubsystem));
        NamedCommands.registerCommand("reset", new ParallelCommandGroup(
                turretLeftSubsystem.setTurretPosCMD(0), shooterLeftSubsystem.SetVelocityCMD(0),
                turretRightSubsystem.setTurretPosCMD(0), shooterRightSubsystem.SetVelocityCMD(0)));

        turretLeftConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake);
        turretLeftConfig.encoder
                // Your factor: 7.2 deg per motor-encoder unit
                .positionConversionFactor(7.2)
                .velocityConversionFactor(7.2);

        turretLeftMotor.configure(turretLeftConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        turretRightConfig
                .inverted(true)
                .idleMode(IdleMode.kBrake);
        turretRightConfig.encoder
                // Your factor: 7.2 deg per motor-encoder unit
                .positionConversionFactor(7.2)
                .velocityConversionFactor(7.2);

        turretRightMotor.configure(turretRightConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        configureBindings();
        CommandScheduler.getInstance().registerSubsystem(intakeSubsystem);
        CommandScheduler.getInstance().registerSubsystem(transferSubsystem);
        CommandScheduler.getInstance().registerSubsystem(shooterLeftSubsystem);
        CommandScheduler.getInstance().registerSubsystem(turretLeftSubsystem);
        CommandScheduler.getInstance().registerSubsystem(shooterRightSubsystem);
        CommandScheduler.getInstance().registerSubsystem(turretRightSubsystem);

    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
                // Drivetrain will execute this command periodically
                drivetrain.applyRequest(
                        () -> drive.withVelocityX(-joystick.getLeftY() * MaxSpeed * (1 - joystick.getRawAxis(2) * .8)) // Drive
                                                                                                                       // forward
                                                                                                                       // with
                                                                                                                       // negative
                                                                                                                       // Y
                                                                                                                       // (forward)
                                .withVelocityY(-joystick.getLeftX() * MaxSpeed * (1 - joystick.getRawAxis(2) * .8)) // Drive
                                                                                                                    // left
                                                                                                                    // with
                                                                                                                    // negative
                                                                                                                    // X
                                                                                                                    // (left)
                                .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise
                                                                                            // with negative X (left)
                ));

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
                drivetrain.applyRequest(() -> idle).ignoringDisable(true));

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        /*
         * joystick.b().whileTrue(drivetrain.applyRequest(() ->
         * point.withModuleDirection(new Rotation2d(-joystick.getLeftY(),
         * -joystick.getLeftX()))
         * ));
         */

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.

        /*
         * joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction
         * .kForward));
         * joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction
         * .kReverse));
         * joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(
         * Direction.kForward));
         * joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(
         * Direction.kReverse));
         */

        // reset the field-centric heading on left bumper press
        // joystick.leftBumper().onTrue(drivetrain.runOnce(() ->
        // drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);

        joystick.rightBumper().onTrue(new ParallelCommandGroup(
                intakeSubsystem.pivotDownCMD(),
                intakeSubsystem.IntakeinCMD(intakeSubsystem)));

        joystick.rightBumper().onFalse(intakeSubsystem.pivotDownSafeCMD());
        joystick.rightBumper().onFalse(intakeSubsystem.IntakeStopCMD(intakeSubsystem));

        new JoystickButton(buttonBoard, 8).onTrue(new ParallelCommandGroup(
                intakeSubsystem.pivotDownCMD(),
                intakeSubsystem.IntakeinCMD(intakeSubsystem)));

        new JoystickButton(buttonBoard, 8).onFalse(new ParallelCommandGroup(
                intakeSubsystem.pivotDownSafeCMD(),
                intakeSubsystem.IntakeStopCMD(intakeSubsystem)));

        Trigger povUp = new Trigger(() -> joystick.getHID().getPOV() == 1);
        // Trigger povDown = new Trigger(() -> joystick.getHID().getPOV() == 4);
        Trigger povRight = new Trigger(() -> joystick.getHID().getPOV() == 2);
        Trigger povLeft = new Trigger(() -> joystick.getHID().getPOV() == 8);

        joystick.pov(0).onTrue(intakeSubsystem.pivotUpCMD()); // no estuvo detectando el POV de xbox, esta forma si jala
        povUp.onTrue(intakeSubsystem.pivotUpCMD());
        new JoystickButton(buttonBoard, 7).onTrue(intakeSubsystem.pivotUpCMD());

        joystick.leftBumper().onTrue(transferSubsystem.TransferShootCMD(transferSubsystem));
        joystick.leftBumper().onFalse(transferSubsystem.StopTransferCMD(transferSubsystem));

        new JoystickButton(buttonBoard, 9).onTrue(transferSubsystem.TransferShootCMD(transferSubsystem));
        new JoystickButton(buttonBoard, 9).onFalse(transferSubsystem.StopTransferCMD(transferSubsystem));

        joystick.rightStick().onTrue(turretLeftSubsystem.setTurretPosCMD(0));
        joystick.leftStick().onTrue(turretLeftSubsystem.setTurretPosCMD(89));

        povRight.onTrue(shooterLeftSubsystem.SetVelocityCMD(100));
        povLeft.onTrue(shooterRightSubsystem.SetVelocityCMD(17));

        joystick.a().onTrue(turretLeftSubsystem.resetOffsetCMD());

        joystick.x().onTrue(new SOTFCommand(drivetrain, shooterLeftSubsystem, turretLeftSubsystem));
        joystick.x().onTrue(new SOTFCommand(drivetrain, shooterRightSubsystem, turretRightSubsystem));
        new JoystickButton(buttonBoard, 3)
                .onTrue(new SOTFCommand(drivetrain, shooterLeftSubsystem, turretLeftSubsystem));
        new JoystickButton(buttonBoard, 3)
                .onTrue(new SOTFCommand(drivetrain, shooterRightSubsystem, turretRightSubsystem));

        joystick.x().onFalse(new ParallelCommandGroup(
                turretLeftSubsystem.setTurretPosCMD(0),
                turretRightSubsystem.setTurretPosCMD(0)));
        new JoystickButton(buttonBoard, 3).onFalse(
                new ParallelCommandGroup(
                        turretLeftSubsystem.setTurretPosCMD(0),
                        turretRightSubsystem.setTurretPosCMD(0)));

        joystick.y().onTrue(new InstantCommand(() -> drivetrain.resetRotation(new Rotation2d(0))));

    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }
}

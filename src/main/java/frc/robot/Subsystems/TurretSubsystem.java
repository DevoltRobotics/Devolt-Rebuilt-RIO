// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * Turret subsystem that:
 *  - Holds an internal desiredAngle (degrees)
 *  - Uses TurretCoordinator to produce a REAL safe setpoint every loop
 *  - Enforces hard limits at motor output
 *
 * IMPORTANT:
 *  - You should construct ONE TurretCoordinator instance and pass it to both turrets.
 *  - Each turret must also get its Side (LEFT or RIGHT).
 */
public class TurretSubsystem extends SubsystemBase {

  private final SparkMax turretMotor;
  private final SparkMaxConfig turretConfig = new SparkMaxConfig();

  public final Translation2d turretOffset;

  private final TurretCoordinator coordinator;
  private final TurretCoordinator.Side side;

  private final PIDController turretPidController = new PIDController(.01, 0, 0);

  // Desired angle requested by commands (degrees, can be any representation)
  private double desiredAngleDeg = 0.0;

  // Output to motor (clamped)
  private double turretOut = 0.0;

  // Encoder offset
  private double turretOffsetEnc = 0.0;

  // Current turret position relative to offset (degrees)
  private double turretRelativePosDeg = 0.0;

  // What we actually command after coordinator + limit normalization (degrees)
  private double commandedSetpointDeg = 0.0;

  // Limits (degrees)
  private double upperLimitDeg = 270.0;
  private double lowerLimitDeg = -90.0;

  // ---------------- Mechanism2d ----------------
  private final LoggedMechanism2d mech = new LoggedMechanism2d(3, 3);
  private final LoggedMechanismRoot2d root = mech.getRoot("TurretRoot", 1.5, 1.5);

  // Blue = actual position
  private final LoggedMechanismLigament2d turretLigament = root.append(new LoggedMechanismLigament2d(
      "TurretActual",
      1.0,
      0,
      6,
      new Color8Bit(Color.kBlue)));

  // Red = target setpoint
  private final LoggedMechanismLigament2d targetLigament = root.append(new LoggedMechanismLigament2d(
      "TurretTarget",
      1.2,
      0,
      4,
      new Color8Bit(Color.kRed)));

  // Green / Orange = limits (visual)
  private final LoggedMechanismLigament2d upperLimitLigament = root.append(new LoggedMechanismLigament2d(
      "UpperLimit",
      1.35,
      90,
      2,
      new Color8Bit(Color.kGreen)));

  private final LoggedMechanismLigament2d lowerLimitLigament = root.append(new LoggedMechanismLigament2d(
      "LowerLimit",
      1.35,
      -270,
      2,
      new Color8Bit(Color.kOrange)));
  // ------------------------------------------------

  public TurretSubsystem(
      int canId,
      TurretCoordinator coordinator,
      TurretCoordinator.Side side,
      Translation2d turretOffset
  ) {
    turretMotor = new SparkMax(canId, MotorType.kBrushless);
    this.turretOffset = turretOffset;

    this.coordinator = coordinator;
    this.side = side;

    turretConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake);
    turretConfig.encoder
        // Your factor: 7.2 deg per motor-encoder unit
        .positionConversionFactor(7.2)
        .velocityConversionFactor(7.2);

    turretMotor.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    setName("TurretSubsystem" + canId);

    Logger.recordOutput(getName() + "/TurretMechanism2d", mech);

    resetOffsetCMD().schedule();
  }

  @Override
  public void periodic() {
    // 1) Measure current relative position (deg)
    turretRelativePosDeg = turretMotor.getEncoder().getPosition() - turretOffsetEnc;

    // 2) Ask coordinator for the REAL safe setpoint (handles collision + limit-safe target representation)
    Rotation2d current = Rotation2d.fromDegrees(turretRelativePosDeg);
    Rotation2d requested = Rotation2d.fromDegrees(desiredAngleDeg);

    Rotation2d realSetpoint = coordinator.intentToRotate(
        side,
        requested,
        current,
        turretMotor.getEncoder().getVelocity()
    );

    commandedSetpointDeg = realSetpoint.getDegrees();

    // 3) PID
    turretPidController.setSetpoint(commandedSetpointDeg);
    double pidOut = turretPidController.calculate(turretRelativePosDeg);

    // 4) Clamp motor output
    turretOut = MathUtil.clamp(pidOut, -0.5, 0.5);

    // 5) Hard limit enforcement (never drive past limits)
    if (turretRelativePosDeg >= upperLimitDeg && turretOut > 0) {
      turretOut = 0;
    }
    if (turretRelativePosDeg <= lowerLimitDeg && turretOut < 0) {
      turretOut = 0;
    }

    // 6) Apply
    turretMotor.set(turretOut);

    // 7) Visual + telemetry
    turretLigament.setAngle(turretRelativePosDeg);
    targetLigament.setAngle(commandedSetpointDeg);
    upperLimitLigament.setAngle(upperLimitDeg);
    lowerLimitLigament.setAngle(lowerLimitDeg);

    SmartDashboard.putNumber(getName() + "/RelPosDeg", turretRelativePosDeg);
    SmartDashboard.putNumber(getName() + "/DesiredDeg", desiredAngleDeg);
    SmartDashboard.putNumber(getName() + "/CmdSetpointDeg", commandedSetpointDeg);
    SmartDashboard.putNumber(getName() + "/Out", turretOut);
    SmartDashboard.putNumber(getName() + "/LowerLimitDeg", lowerLimitDeg);
    SmartDashboard.putNumber(getName() + "/UpperLimitDeg", upperLimitDeg);
  }

  // ---------------- Public API ----------------

  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(turretRelativePosDeg);
  }

  /** Desired angle in degrees (can be any representation; coordinator will unwrap safely). */
  public void setAngle(double angleDeg) {
    desiredAngleDeg = angleDeg;
  }

  /** Optional: set limits if they differ per turret */
  public void setLimits(double lowerDeg, double upperDeg) {
    lowerLimitDeg = lowerDeg;
    upperLimitDeg = upperDeg;
  }

  public void resetOffset() {
    turretOffsetEnc = turretMotor.getEncoder().getPosition();
  }

  // ---------------- Commands ----------------

  public Command resetOffsetCMD() {
    return new InstantCommand(this::resetOffset, this);
  }

  public Command setTurretPosCMD(double posDeg) {
    return new InstantCommand(() -> setAngle(posDeg), this);
  }
}
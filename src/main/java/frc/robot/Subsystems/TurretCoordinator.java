package frc.robot.Subsystems;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.mechanism.LoggedMechanism2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismLigament2d;
import org.littletonrobotics.junction.mechanism.LoggedMechanismRoot2d;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;

/**
 * TurretCoordinator (2-turret collision coordinator)
 *
 * - Collision model: two oriented ellipses (same a,b) at fixed centers (-d/2,0) and (+d/2,0).
 * - Overlap test: SAT-like with exact ellipse support function, tested on 4 axes (major/minor of each ellipse).
 * - Lock model: if collision predicted in lookahead, hold one turret, let the other finish.
 * - Lookahead: uses measured angular velocity (deg/s).
 * - Visualization: LoggedMechanism2d cross (major/minor) per turret + center link + status colors.
 *
 * How to use:
 *  - Create ONE TurretCoordinator.
 *  - Each turret calls intentToRotate(...) each loop and uses returned setpoint.
 */
public class TurretCoordinator {

  // ===================== Geometry (INCHES) =====================
  public static final double CENTERPOINTS_DISTANCE_IN = 12.0;

  // Full axes lengths (in)
  public static final double TURRET_MAJOR_AXIS_IN = 11.3;
  public static final double TURRET_MINOR_AXIS_IN = 7.0;

  // Encoder-zero to ellipse-major-axis offsets (deg) (tune if needed)
  public static final double PHI_LEFT_DEG  = 0.0;
  public static final double PHI_RIGHT_DEG = 0.0;

  // ===================== Safety / Behavior =====================
  public static final double CLEARANCE_ENTER_IN = 0.75; // start holding when predicted overlap closer than this
  public static final double CLEARANCE_EXIT_IN  = 1.25; // release only after this (hysteresis)

  public static final double ON_TARGET_TOL_DEG = 1.5;

  // Lookahead horizon
  public static final double DT_SEC = 0.02;
  public static final int LOOKAHEAD_STEPS = 4;         // 80ms
  public static final double MAX_LOOKAHEAD_DEG = 25.0; // clamp crazy velocities / spikes

  public enum Side { LEFT, RIGHT }

  // ===================== Fixed centers (inches) =====================
  private final double halfD = CENTERPOINTS_DISTANCE_IN * 0.5;
  private final Translation2d leftCenter  = new Translation2d(-halfD, 0.0);
  private final Translation2d rightCenter = new Translation2d(+halfD, 0.0);

  // Semi-axes (inches)
  private final double a = TURRET_MAJOR_AXIS_IN * 0.5; // semi-major
  private final double b = TURRET_MINOR_AXIS_IN * 0.5; // semi-minor

  // ===================== Latest state =====================
  private Rotation2d leftTarget   = new Rotation2d();
  private Rotation2d rightTarget  = new Rotation2d();
  private Rotation2d leftCurrent  = new Rotation2d();
  private Rotation2d rightCurrent = new Rotation2d();

  private double leftOmegaDegPerSec  = 0.0;
  private double rightOmegaDegPerSec = 0.0;

  // Outputs
  private Rotation2d leftCmd  = new Rotation2d();
  private Rotation2d rightCmd = new Rotation2d();

  // Lock state
  private boolean locked = false;
  private Side heldSide = null;

  // ===================== LoggedMechanism2d =====================
  private static final double VIEW_W = 30.0;
  private static final double VIEW_H = 20.0;
  private static final double SCALE = 1.0; // 1 mech unit ~ 1 inch

  private final LoggedMechanism2d mech = new LoggedMechanism2d(VIEW_W, VIEW_H);

  private final LoggedMechanismRoot2d leftRoot;
  private final LoggedMechanismRoot2d rightRoot;

  private final LoggedMechanismLigament2d centerLink;

  private final LoggedMechanismLigament2d leftMajor;
  private final LoggedMechanismLigament2d leftMinor;
  private final LoggedMechanismLigament2d rightMajor;
  private final LoggedMechanismLigament2d rightMinor;

  private final LoggedMechanismLigament2d leftDot;
  private final LoggedMechanismLigament2d rightDot;

  private static final Color8Bit C_ACTIVE  = new Color8Bit(Color.kBlue);
  private static final Color8Bit C_WAIT    = new Color8Bit(Color.kGray);
  private static final Color8Bit C_COLLIDE = new Color8Bit(Color.kRed);
  private static final Color8Bit C_LINK    = new Color8Bit(Color.kWhite);
  private static final Color8Bit C_DOT     = new Color8Bit(Color.kYellow);

  public TurretCoordinator() {
    double d = CENTERPOINTS_DISTANCE_IN * SCALE;

    // Place centers on canvas
    leftRoot  = mech.getRoot("LeftTurretCenter",  VIEW_W / 2.0 - d / 2.0, VIEW_H / 2.0);
    rightRoot = mech.getRoot("RightTurretCenter", VIEW_W / 2.0 + d / 2.0, VIEW_H / 2.0);

    // Link between centers (for context)
    centerLink = leftRoot.append(new LoggedMechanismLigament2d(
        "CenterLink", d, 0.0, 2.0, C_LINK
    ));

    // Dots
    leftDot = leftRoot.append(new LoggedMechanismLigament2d("LeftDot", 0.5, 0.0, 6.0, C_DOT));
    rightDot = rightRoot.append(new LoggedMechanismLigament2d("RightDot", 0.5, 0.0, 6.0, C_DOT));

    // Radii for cross (semi axes)
    double aMech = a * SCALE;
    double bMech = b * SCALE;

    leftMajor = leftRoot.append(new LoggedMechanismLigament2d("LeftMajor", aMech, 0.0, 4.0, C_WAIT));
    leftMinor = leftRoot.append(new LoggedMechanismLigament2d("LeftMinor", bMech, 90.0, 4.0, C_WAIT));

    rightMajor = rightRoot.append(new LoggedMechanismLigament2d("RightMajor", aMech, 0.0, 4.0, C_WAIT));
    rightMinor = rightRoot.append(new LoggedMechanismLigament2d("RightMinor", bMech, 90.0, 4.0, C_WAIT));

    // Record mechanism (updates propagate when you mutate ligaments)
    Logger.recordOutput("TurretCoordinator/Mechanism2d", mech);
  }

  /**
   * Main API: returns the real setpoint this turret should follow (may HOLD current).
   *
   * @param turret which turret is calling
   * @param target desired turret angle
   * @param current measured turret angle
   * @param omegaDegPerSec measured angular velocity (deg/s). Positive = increasing angle.
   */
  public Rotation2d intentToRotate(Side turret, Rotation2d target, Rotation2d current, double omegaDegPerSec) {
    if (turret == Side.LEFT) {
      leftTarget = target;
      leftCurrent = current;
      leftOmegaDegPerSec = omegaDegPerSec;
    } else {
      rightTarget = target;
      rightCurrent = current;
      rightOmegaDegPerSec = omegaDegPerSec;
    }

    recompute();
    updateMechanismView();

    return (turret == Side.LEFT) ? leftCmd : rightCmd;
  }

  // ===================== Coordinator logic =====================

  private void recompute() {
    Rotation2d desiredL = leftTarget;
    Rotation2d desiredR = rightTarget;

    Rotation2d predL = predictAngle(leftCurrent, leftTarget, leftOmegaDegPerSec);
    Rotation2d predR = predictAngle(rightCurrent, rightTarget, rightOmegaDegPerSec);

    boolean collideEnter = ellipsesOverlap(predL, predR, CLEARANCE_ENTER_IN);
    boolean collideExit  = ellipsesOverlap(predL, predR, CLEARANCE_EXIT_IN);

    if (!locked) {
      if (collideEnter) {
        Side mover = chooseMover();
        heldSide = (mover == Side.LEFT) ? Side.RIGHT : Side.LEFT;
        locked = true;
      }
    } else {
      if (!collideExit) {
        locked = false;
        heldSide = null;
      }
    }

    if (locked && heldSide != null) {
      if (heldSide == Side.LEFT)  desiredL = leftCurrent;   // HOLD
      if (heldSide == Side.RIGHT) desiredR = rightCurrent;  // HOLD
    }

    leftCmd = desiredL;
    rightCmd = desiredR;
  }

  private Side chooseMover() {
    double eL = Math.abs(angleDiffDeg(leftTarget, leftCurrent));
    double eR = Math.abs(angleDiffDeg(rightTarget, rightCurrent));

    boolean lOnTarget = eL <= ON_TARGET_TOL_DEG;
    boolean rOnTarget = eR <= ON_TARGET_TOL_DEG;

    if (lOnTarget && !rOnTarget) return Side.RIGHT;
    if (rOnTarget && !lOnTarget) return Side.LEFT;

    return (eL <= eR) ? Side.LEFT : Side.RIGHT;
  }

  private Rotation2d predictAngle(Rotation2d current, Rotation2d target, double omegaDegPerSec) {
    double horizonSec = DT_SEC * LOOKAHEAD_STEPS;

    double delta = omegaDegPerSec * horizonSec;
    delta = clamp(delta, -MAX_LOOKAHEAD_DEG, MAX_LOOKAHEAD_DEG);

    Rotation2d rawFuture = current.plus(Rotation2d.fromDegrees(delta));

    // Keep prediction sane: don't predict opposite the target direction
    double toTarget = angleDiffDeg(target, current);
    double toFuture = angleDiffDeg(rawFuture, current);

    if (Math.signum(toFuture) != 0.0 && Math.signum(toTarget) != 0.0
        && Math.signum(toFuture) != Math.signum(toTarget)) {
      return current;
    }

    // Don't overshoot target in prediction
    if (Math.abs(toFuture) > Math.abs(toTarget)) {
      return target;
    }

    return rawFuture;
  }

  // ===================== Geometry: ellipse overlap (SAT-like) =====================

  private boolean ellipsesOverlap(Rotation2d thetaLRaw, Rotation2d thetaRRaw, double clearanceIn) {
    // apply shape-axis offsets
    Rotation2d thetaL = thetaLRaw.plus(Rotation2d.fromDegrees(PHI_LEFT_DEG));
    Rotation2d thetaR = thetaRRaw.plus(Rotation2d.fromDegrees(PHI_RIGHT_DEG));

    // Candidate axes: major/minor of each ellipse (unit vectors)
    Translation2d[] axes = new Translation2d[] {
        axisMajor(thetaL),
        axisMinor(thetaL),
        axisMajor(thetaR),
        axisMinor(thetaR)
    };

    double clearanceHalf = clearanceIn * 0.5;

    for (Translation2d axis : axes) {
      double p1 = dot(leftCenter, axis);
      double p2 = dot(rightCenter, axis);

      double r1 = supportEllipseOnAxis(axis, thetaL) + clearanceHalf;
      double r2 = supportEllipseOnAxis(axis, thetaR) + clearanceHalf;

      double min1 = p1 - r1, max1 = p1 + r1;
      double min2 = p2 - r2, max2 = p2 + r2;

      // separating axis found => no overlap
      if (max1 < min2 || max2 < min1) return false;
    }

    // overlaps on all tested axes => treat as collision
    return true;
  }

  private Translation2d axisMajor(Rotation2d theta) {
    return new Translation2d(1.0, 0.0).rotateBy(theta);
  }

  private Translation2d axisMinor(Rotation2d theta) {
    return new Translation2d(0.0, 1.0).rotateBy(theta);
  }

  /** Exact ellipse support radius on an axis (axis is unit in world). */
  private double supportEllipseOnAxis(Translation2d axisWorldUnit, Rotation2d theta) {
    // axisLocal = R(-theta) * axisWorld
    Translation2d axisLocal = axisWorldUnit.rotateBy(theta.unaryMinus());
    double ux = axisLocal.getX();
    double uy = axisLocal.getY();
    return Math.hypot(a * ux, b * uy);
  }

  private double dot(Translation2d a, Translation2d b) {
    return a.getX() * b.getX() + a.getY() * b.getY();
  }

  // ===================== Mechanism2d view =====================

  private void updateMechanismView() {
    // Current angles in degrees
    double lDeg = leftCurrent.getDegrees();
    double rDeg = rightCurrent.getDegrees();

    // Update cross angles
    leftMajor.setAngle(lDeg);
    leftMinor.setAngle(lDeg + 90.0);
    rightMajor.setAngle(rDeg);
    rightMinor.setAngle(rDeg + 90.0);

    // Recompute predicted collision (for coloring)
    Rotation2d predL = predictAngle(leftCurrent, leftTarget, leftOmegaDegPerSec);
    Rotation2d predR = predictAngle(rightCurrent, rightTarget, rightOmegaDegPerSec);

    boolean willCollideEnter = ellipsesOverlap(predL, predR, CLEARANCE_ENTER_IN);

    // Base colors: who is moving (not held) vs held
    boolean leftHeld = locked && heldSide == Side.LEFT;
    boolean rightHeld = locked && heldSide == Side.RIGHT;

    Color8Bit leftColor = leftHeld ? C_WAIT : C_ACTIVE;
    Color8Bit rightColor = rightHeld ? C_WAIT : C_ACTIVE;

    // If predicted collision right now, show red to indicate danger region
    if (willCollideEnter) {
      // show BOTH red if still in danger, easier to read
      leftColor = C_COLLIDE;
      rightColor = C_COLLIDE;
    }

    leftMajor.setColor(leftColor);
    leftMinor.setColor(leftColor);
    rightMajor.setColor(rightColor);
    rightMinor.setColor(rightColor);

    // Keep link and dots visible
    centerLink.setColor(C_LINK);
    leftDot.setColor(C_DOT);
    rightDot.setColor(C_DOT);

    // Optional logs for AdvantageScope
    Logger.recordOutput("TurretCoordinator/Locked", locked);
    Logger.recordOutput("TurretCoordinator/HeldSide", heldSide == null ? "NONE" : heldSide.name());
    Logger.recordOutput("TurretCoordinator/LeftCurrentDeg", lDeg);
    Logger.recordOutput("TurretCoordinator/RightCurrentDeg", rDeg);
    Logger.recordOutput("TurretCoordinator/LeftTargetDeg", leftTarget.getDegrees());
    Logger.recordOutput("TurretCoordinator/RightTargetDeg", rightTarget.getDegrees());
    Logger.recordOutput("TurretCoordinator/WillCollideLookahead", willCollideEnter);
  }

  // ===================== Angle helpers =====================

  /** Smallest signed difference target-current in degrees, wrapped to [-180, 180]. */
  private static double angleDiffDeg(Rotation2d target, Rotation2d current) {
    double diff = target.minus(current).getDegrees();
    diff = ((diff + 180.0) % 360.0 + 360.0) % 360.0 - 180.0;
    return diff;
  }

  private static double clamp(double v, double lo, double hi) {
    return Math.max(lo, Math.min(hi, v));
  }
}
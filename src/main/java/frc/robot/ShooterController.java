package frc.robot;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.NavigableMap;
import java.util.TreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShooterController {

    // Distance (m) -> {RPS, timeOfFlight (s)}
    private static final HashMap<Double, ShooterParams> SHOOTER_MAP = new HashMap<>();

    // Interpolation for distance -> ShooterParams
    private static final InterpolatingTreeMap<Double, ShooterParams> SHOOTER_INTERP_MAP =
        new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            (start, end, t) -> new ShooterParams(
                MathUtil.interpolate(start.rps, end.rps, t),
                MathUtil.interpolate(start.timeOfFlight, end.timeOfFlight, t)
            )
        );

    /**
     * Reverse map: (horizontal muzzle velocity estimate) -> distance
     * Must be ORDERED, so TreeMap, not HashMap.
     */
    private static final NavigableMap<Double, Double> VELOCITY_TO_DISTANCE = new TreeMap<>();

    // LUT bounds (set to your min/max distance keys)
    private static final double MIN_DISTANCE = 2.0;
    private static final double MAX_DISTANCE = 9.3;

    static {
        SHOOTER_MAP.put(2.0,  new ShooterParams(45.5, 0.6));
        SHOOTER_MAP.put(2.5,  new ShooterParams(46.0, 0.83));
        SHOOTER_MAP.put(3.0,  new ShooterParams(48.5, 0.88));
        SHOOTER_MAP.put(3.5,  new ShooterParams(51.3, 0.93));
        SHOOTER_MAP.put(4.0,  new ShooterParams(55.7, 1.08));
        SHOOTER_MAP.put(4.5,  new ShooterParams(59.7, 1.16));
        SHOOTER_MAP.put(5.0,  new ShooterParams(63.7, 01.24));
        SHOOTER_MAP.put(5.5,  new ShooterParams(66.4, 1.34));
        SHOOTER_MAP.put(6.0,  new ShooterParams(69.4, 1.53));
        SHOOTER_MAP.put(9.3,  new ShooterParams(100.0, 2.18));



        // Fill distance interpolation map
        for (Entry<Double, ShooterParams> e : SHOOTER_MAP.entrySet()) {
            SHOOTER_INTERP_MAP.put(e.getKey(), e.getValue());
        }

        // Build ordered reverse LUT: velocity -> distance
        // Using vel = distance / ToF as your "baseline horizontal velocity" for each entry.
        for (Entry<Double, ShooterParams> e : SHOOTER_MAP.entrySet()) {
            double dist = e.getKey();
            double tof  = e.getValue().timeOfFlight;
            double vel  = dist / tof;
            VELOCITY_TO_DISTANCE.put(vel, dist);
        }
    }

    public static ShooterResult calculate(
        Translation2d robotPosition,
        Translation2d robotVelocity,
        Translation2d goalPosition,
        double latencyCompensation) {

    // 1) Future position (optional)
    Translation2d futurePos = robotPosition.plus(robotVelocity.times(latencyCompensation));

    // 2) Vector to goal in FIELD frame
    Translation2d toGoal = goalPosition.minus(futurePos);
    double distance = toGoal.getNorm();

    // Guard
    if (distance < 1e-6) {
        return new ShooterResult(new Rotation2d(), 0.0);
    }

    // 3) Clamp distance to LUT range & get baseline params from LUT
    double clampedDist = MathUtil.clamp(distance, MIN_DISTANCE, MAX_DISTANCE);
    ShooterParams baseline = SHOOTER_INTERP_MAP.get(clampedDist);
    if (baseline == null) {
        baseline = SHOOTER_INTERP_MAP.get(MIN_DISTANCE);
    }

    // 4) Baseline horizontal speed (your model)
    double baselineVelocity = clampedDist / baseline.timeOfFlight;

    // 5) Direction to goal
    Translation2d targetDirection = toGoal.div(distance);

    // 6) Build desired target velocity vector toward goal (for ANGLE)
    Translation2d targetVelocity = targetDirection.times(baselineVelocity);

    // 7) SOTF vector compensation -> turret angle (2D)
    Translation2d shotVelocity = targetVelocity.minus(robotVelocity);
    Rotation2d turretFieldAngle = shotVelocity.getAngle();

    // ------------------ FIX RÁPIDO PARA RPS ------------------
    // Solo compensa RPS por componente radial (acercarte/alejarte),
    // no por magnitud completa (evita spikes a 100 RPS).
    double vRadial = robotVelocity.getX() * targetDirection.getX()
                   + robotVelocity.getY() * targetDirection.getY();

    // Required horizontal speed along line-to-goal
    double requiredVelocity1D = baselineVelocity - vRadial;

    // Limita cuánto puede cambiar la velocidad por movimiento del robot, para evitar spikes en RPS.
    double maxExtraMps = 3.5; 
    requiredVelocity1D = MathUtil.clamp(
        requiredVelocity1D,
        baselineVelocity - maxExtraMps,
        baselineVelocity + maxExtraMps
    );
 
    // Convierte velocidad->distancia efectiva->RPS usando tu LUT existente
    double effectiveDistance = velocityToEffectiveDistance(requiredVelocity1D);
    double clampedEff = MathUtil.clamp(effectiveDistance, MIN_DISTANCE, MAX_DISTANCE);
    double requiredRps = SHOOTER_INTERP_MAP.get(clampedEff).rps;
    // ---------------------------------------------------------

    return new ShooterResult(turretFieldAngle, requiredRps);
}

    /**
     * Converts a required horizontal muzzle velocity to an "effective distance"
     * using an ORDERED velocity->distance LUT, with interpolation.
     */
    public static double velocityToEffectiveDistance(double velocity) {
        // Find nearest velocities around the requested value
        Map.Entry<Double, Double> floor = VELOCITY_TO_DISTANCE.floorEntry(velocity);
        Map.Entry<Double, Double> ceil  = VELOCITY_TO_DISTANCE.ceilingEntry(velocity);

        if (floor == null) {
            return VELOCITY_TO_DISTANCE.firstEntry().getValue();
        }
        if (ceil == null) {
            return VELOCITY_TO_DISTANCE.lastEntry().getValue();
        }
        if (ceil.getKey().equals(floor.getKey())) {
            return ceil.getValue();
        }

        // Interpolate distance between the two surrounding velocity keys
        double v0 = floor.getKey();
        double d0 = floor.getValue();
        double v1 = ceil.getKey();
        double d1 = ceil.getValue();

        double t = (velocity - v0) / (v1 - v0);
        return MathUtil.interpolate(d0, d1, t);
    }

    public static double calculateAdjustedRps(double requiredVelocity) {
        double effectiveDistance = velocityToEffectiveDistance(requiredVelocity);
        double clampedEff = MathUtil.clamp(effectiveDistance, MIN_DISTANCE, MAX_DISTANCE);
        return SHOOTER_INTERP_MAP.get(clampedEff).rps;
    }

    public static record ShooterParams(double rps, double timeOfFlight) {}

    public static record ShooterResult(Rotation2d turretFieldAngle, double requiredRps) {}
}
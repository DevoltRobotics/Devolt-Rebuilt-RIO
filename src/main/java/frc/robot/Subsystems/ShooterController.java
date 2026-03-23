package frc.robot.Subsystems;

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
    private static final double MIN_DISTANCE = 1.971;
    private static final double MAX_DISTANCE = 5.818;

    static {
        SHOOTER_MAP.put(1.971,  new ShooterParams(40, 0.83));
        SHOOTER_MAP.put(2.585,  new ShooterParams(44, 1.4));
        SHOOTER_MAP.put(3.143,  new ShooterParams(48, 1.13));
        SHOOTER_MAP.put(3.346,  new ShooterParams(52, 0.98));
        SHOOTER_MAP.put(3.779,  new ShooterParams(56.5, 1.37));
        SHOOTER_MAP.put(4.143,  new ShooterParams(61, 1.29));
        SHOOTER_MAP.put(4.593,  new ShooterParams(62.0, 1.42));
        SHOOTER_MAP.put(5.2,  new ShooterParams(64, 1.72));
        SHOOTER_MAP.put(5.818,  new ShooterParams(68, 1.79));
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
        double latencyCompensation
    ) {

    // 1) Future position (optional)
    Translation2d futurePos = robotPosition.plus(robotVelocity.times(latencyCompensation));

    // 2) Vector to goal in FIELD frame
    Translation2d toGoal = goalPosition.minus(futurePos);
    double distance = toGoal.getNorm();

    // Guard
    if (distance < 1e-6) {
        return new ShooterResult(new Rotation2d(), 0.0, 0.0);
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
    double requiredRpsSOTF = SHOOTER_INTERP_MAP.get(clampedEff).rps;
    double requiredRps = SHOOTER_INTERP_MAP.get(clampedDist).rps;

    // ---------------------------------------------------------

    return new ShooterResult(turretFieldAngle, requiredRps, distance);
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

    public static record ShooterResult(Rotation2d turretFieldAngle, double requiredRps, double distanceToGoal) {}
}
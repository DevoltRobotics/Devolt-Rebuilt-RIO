package frc.robot.Subsystems;

import java.util.HashMap;
import java.util.Map.Entry;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.math.interpolation.InverseInterpolator;

public class ShooterController {

    // Distance (m) -> {RPS, timeOfFlight (s)}
    private static final HashMap<Double, ShooterParams> SHOOTER_MAP = new HashMap<>();

    // Interpolación: distancia -> ShooterParams
    private static final InterpolatingTreeMap<Double, ShooterParams> SHOOTER_INTERP_MAP =
        new InterpolatingTreeMap<>(
            InverseInterpolator.forDouble(),
            (start, end, t) -> new ShooterParams(
                MathUtil.interpolate(start.rps, end.rps, t),
                MathUtil.interpolate(start.timeOfFlight, end.timeOfFlight, t)
            )
        );

    private static final double MIN_DISTANCE = 1.971;
    private static final double MAX_DISTANCE = 5.818;

    static {
           SHOOTER_MAP.put(1.971,  new ShooterParams(40, 0.83));
        SHOOTER_MAP.put(2.585,  new ShooterParams(46, 1.4));
        SHOOTER_MAP.put(3.143,  new ShooterParams(51, 1.13));
        SHOOTER_MAP.put(3.346,  new ShooterParams(53, 0.98));
        SHOOTER_MAP.put(3.779,  new ShooterParams(57, 1.37));
        SHOOTER_MAP.put(4.143,  new ShooterParams(62, 1.29));
        SHOOTER_MAP.put(4.593,  new ShooterParams(63.0, 1.42));
        SHOOTER_MAP.put(5.2,  new ShooterParams(64, 1.72));
        SHOOTER_MAP.put(5.818,  new ShooterParams(68, 1.79));

        for (Entry<Double, ShooterParams> e : SHOOTER_MAP.entrySet()) {
            SHOOTER_INTERP_MAP.put(e.getKey(), e.getValue());
        }
    }

    public static ShooterResult calculate(
            Translation2d robotPosition,
            Translation2d robotVelocity,
            Translation2d goalPosition,
            double latencyCompensation) {

        // 1) Compensación inicial por latencia
        Translation2d latencyAdjustedPos =
            robotPosition.plus(robotVelocity.times(latencyCompensation));

        // 2) Primera estimación de distancia al goal
        Translation2d toGoal = goalPosition.minus(latencyAdjustedPos);
        double distance = toGoal.getNorm();

        if (distance < 1e-6) {
            return new ShooterResult(new Rotation2d(), 0.0, 0.0, 0.0, 0.0);
        }

        // 3) Obtener baseline inicial del LUT
        double clampedDist = MathUtil.clamp(distance, MIN_DISTANCE, MAX_DISTANCE);
        ShooterParams baseline = SHOOTER_INTERP_MAP.get(clampedDist);
        if (baseline == null) {
            baseline = SHOOTER_INTERP_MAP.get(MIN_DISTANCE);
        }

        double tof = baseline.timeOfFlight;

        // 4) Iterar para refinar distancia y ToF mientras el robot se mueve
        double prevDist = distance;

        for (int i = 0; i < 10; i++) {
            Translation2d predictedRobotPos =
                latencyAdjustedPos.plus(robotVelocity.times(tof));

            toGoal = goalPosition.minus(predictedRobotPos);
            distance = toGoal.getNorm();

            if (distance < 1e-6) {
                return new ShooterResult(new Rotation2d(), 0.0, 0.0, 0.0, 0.0);
            }

            clampedDist = MathUtil.clamp(distance, MIN_DISTANCE, MAX_DISTANCE);
            baseline = SHOOTER_INTERP_MAP.get(clampedDist);
            if (baseline == null) {
                baseline = SHOOTER_INTERP_MAP.get(MIN_DISTANCE);
            }

            tof = baseline.timeOfFlight;

            if (Math.abs(distance - prevDist) < 0.01) {
                break;
            }

            prevDist = distance;
        }

        // 5) Dirección refinada al goal
        Translation2d targetDirection = toGoal.div(distance);

        // 6) Velocidad base horizontal según tu LUT
        double baselineVelocity = clampedDist / baseline.timeOfFlight;

        // 7) Vector deseado de la pelota en field frame
        Translation2d targetVelocity = targetDirection.times(baselineVelocity);

        // 8) Compensación vectorial completa para la torreta
        Translation2d shotVelocity = targetVelocity.minus(robotVelocity);
        Rotation2d turretFieldAngle = shotVelocity.getAngle();

        // 9) RPS base del LUT
        double baseRps = baseline.rps;

        // 10) Ajuste pequeño SOLO por componente radial
        double vRadial = robotVelocity.getX() * targetDirection.getX()
                       + robotVelocity.getY() * targetDirection.getY();

        // Tuning
        double kRpsPerMps = 1.5;
        double maxDeltaRps = 8.0;

        double deltaRps = MathUtil.clamp(
            -vRadial * kRpsPerMps,
            -maxDeltaRps,
            maxDeltaRps
        );

        double requiredRps = baseRps + deltaRps;

        // 11) Clamp final de seguridad
        requiredRps = MathUtil.clamp(requiredRps, 0.0, 100.0);

        return new ShooterResult(
            turretFieldAngle,
            requiredRps,
            distance,
            tof,
            vRadial
        );
    }

    public static record ShooterParams(double rps, double timeOfFlight) {}

    public static record ShooterResult(
        Rotation2d turretFieldAngle,
        double requiredRps,
        double distanceToGoal,
        double timeOfFlight,
        double radialVelocity
    ) {}
}
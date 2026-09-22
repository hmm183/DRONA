package drona.deadreckoning.support

import kotlin.math.cos
import kotlin.math.sin

/**
 * Deterministic ground-truth drive generator.
 *
 * Produces a trajectory at a fixed rate, then derives per-window displacement targets
 * *exactly the way the training pipeline does* in `windowing.py` and
 * `global_to_vehicle_frame`, so the estimator can be fed perfect model predictions.
 *
 * That separation matters: if a perfect prediction still fails to reconstruct the
 * trajectory, the defect is in the fusion mathematics rather than in the network. This
 * is how the heading double-count and the start-of-window frame error are isolated
 * without needing the IO-VNBD dataset.
 */
object SyntheticDrive {

    const val SAMPLE_RATE_HZ = 10
    const val WINDOW_SAMPLES = 20

    /** One leg of a drive: constant speed and constant yaw rate. */
    data class Segment(
        val seconds: Double,
        val speedMps: Double,
        val yawRateDegPerSec: Double
    )

    /** Ground-truth pose at one sample instant. */
    data class Pose(
        val timeSeconds: Double,
        val eastMeters: Double,
        val northMeters: Double,
        val headingDegrees: Double,
        val speedMps: Double,
        val yawRateRadPerSec: Double
    )

    /**
     * One window's worth of what a perfect model would emit, plus the gyro samples that
     * physically occurred during that window.
     */
    data class Window(
        val startIndex: Int,
        val endIndex: Int,
        val forwardMeters: Double,
        val lateralMeters: Double,
        val headingDeltaRadians: Double,
        val gyroYawRates: List<Double>,
        val spanSeconds: Double
    )

    data class Truth(val poses: List<Pose>, val windows: List<Window>) {
        val finalPose: Pose get() = poses.last()

        /** Straight-line distance actually travelled along the path. */
        val pathLengthMeters: Double
            get() = poses.zipWithNext().sumOf { (a, b) ->
                val de = b.eastMeters - a.eastMeters
                val dn = b.northMeters - a.northMeters
                kotlin.math.sqrt(de * de + dn * dn)
            }
    }

    /**
     * Integrate the segments into a ground-truth pose sequence.
     *
     * ENU convention, heading clockwise from North:
     *   east  += v * sin(heading) * dt
     *   north += v * cos(heading) * dt
     *
     * A positive yaw rate increases heading, which is a right turn. In the vehicle FRD
     * frame that is exactly a positive rate about the Down axis, so the generated gyro
     * samples can be fed straight into the estimator.
     */
    fun generate(
        segments: List<Segment>,
        initialHeadingDegrees: Double = 0.0,
        sampleRateHz: Int = SAMPLE_RATE_HZ,
        windowSamples: Int = WINDOW_SAMPLES
    ): Truth {
        val dt = 1.0 / sampleRateHz
        val poses = mutableListOf<Pose>()

        var time = 0.0
        var east = 0.0
        var north = 0.0
        var headingDegrees = initialHeadingDegrees

        poses += Pose(time, east, north, headingDegrees, segments.first().speedMps, Math.toRadians(segments.first().yawRateDegPerSec))

        segments.forEach { segment ->
            val steps = Math.round(segment.seconds * sampleRateHz).toInt()
            val yawRate = Math.toRadians(segment.yawRateDegPerSec)
            repeat(steps) {
                val headingRadians = Math.toRadians(headingDegrees)
                east += segment.speedMps * sin(headingRadians) * dt
                north += segment.speedMps * cos(headingRadians) * dt
                headingDegrees += segment.yawRateDegPerSec * dt
                time += dt
                poses += Pose(time, east, north, headingDegrees, segment.speedMps, yawRate)
            }
        }

        return Truth(poses, buildWindows(poses, windowSamples, dt))
    }

    /**
     * Non-overlapping windows, mirroring how the runtime consumes the model. Targets are
     * built from ground truth using the start-of-window heading, matching training.
     */
    private fun buildWindows(poses: List<Pose>, windowSamples: Int, dt: Double): List<Window> {
        val windows = mutableListOf<Window>()
        var start = 0
        while (start + windowSamples - 1 < poses.size) {
            val end = start + windowSamples - 1
            val from = poses[start]
            val to = poses[end]

            val deltaEast = to.eastMeters - from.eastMeters
            val deltaNorth = to.northMeters - from.northMeters
            val heading = Math.toRadians(from.headingDegrees)

            val forward = deltaEast * sin(heading) + deltaNorth * cos(heading)
            val lateral = deltaEast * cos(heading) - deltaNorth * sin(heading)
            val headingDelta = Math.toRadians(wrapDegrees(to.headingDegrees - from.headingDegrees))

            // Rates that physically occurred while advancing from start to end.
            val rates = (start until end).map { poses[it + 1].yawRateRadPerSec }

            windows += Window(
                startIndex = start,
                endIndex = end,
                forwardMeters = forward,
                lateralMeters = lateral,
                headingDeltaRadians = headingDelta,
                gyroYawRates = rates,
                spanSeconds = (windowSamples - 1) * dt
            )
            start = end
        }
        return windows
    }

    private fun wrapDegrees(value: Double): Double = ((value + 180.0) % 360.0 + 360.0) % 360.0 - 180.0

    // Convenience drives -----------------------------------------------------------

    /** 30 s straight at 15 m/s, about 450 m. */
    fun straightDrive(seconds: Double = 30.0, speedMps: Double = 15.0) =
        generate(listOf(Segment(seconds, speedMps, 0.0)))

    /** Straight, then a sustained right turn, then straight again. */
    fun turningDrive(speedMps: Double = 12.0, yawRateDegPerSec: Double = 9.0) = generate(
        listOf(
            Segment(seconds = 8.0, speedMps = speedMps, yawRateDegPerSec = 0.0),
            Segment(seconds = 10.0, speedMps = speedMps, yawRateDegPerSec = yawRateDegPerSec),
            Segment(seconds = 8.0, speedMps = speedMps, yawRateDegPerSec = 0.0)
        )
    )

    /** A full 90 degree right turn at constant radius. */
    fun rightAngleTurn(speedMps: Double = 10.0) = generate(
        listOf(
            Segment(seconds = 4.0, speedMps = speedMps, yawRateDegPerSec = 0.0),
            Segment(seconds = 10.0, speedMps = speedMps, yawRateDegPerSec = 9.0),
            Segment(seconds = 4.0, speedMps = speedMps, yawRateDegPerSec = 0.0)
        )
    )

    /** Stationary for 20 s. */
    fun stationary(seconds: Double = 20.0) = generate(listOf(Segment(seconds, 0.0, 0.0)))
}

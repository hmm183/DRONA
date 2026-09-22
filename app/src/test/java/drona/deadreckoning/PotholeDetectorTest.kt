package drona.deadreckoning

import drona.deadreckoning.adapter.PotholeDetector
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertTrue
import org.junit.Test

class PotholeDetectorTest {

    @Test
    fun `detects pothole on strong impact and gyro spike`() {
        val detector = PotholeDetector()

        val samples = listOf(
            Triple(0f, 0f, 14.5f) to Triple(2.4f, 1.1f, 0.5f),
            Triple(0f, 0f, 16.5f) to Triple(3.0f, 1.6f, 0.7f),
            Triple(0f, 0f, 18.0f) to Triple(3.2f, 1.8f, 0.7f)
        )

        val results = samples.map { (accel, gyro) ->
            detector.update(
                accelX = accel.first,
                accelY = accel.second,
                accelZ = accel.third,
                gyroX = gyro.first,
                gyroY = gyro.second,
                gyroZ = gyro.third
            )
        }

        assertTrue(results.last().detected)
        assertTrue(results.last().severity.contains("Moderate") || results.last().severity.contains("Severe") || results.last().severity.contains("Minor"))
    }

    @Test
    fun `ignores normal road vibration`() {
        val detector = PotholeDetector()

        val result = detector.update(
            accelX = 0.2f,
            accelY = -0.3f,
            accelZ = 9.8f,
            gyroX = 0.1f,
            gyroY = 0.05f,
            gyroZ = 0.08f
        )

        assertFalse(result.detected)
    }

    @Test
    fun `ignores slow hand motion that is not a pothole`() {
        val detector = PotholeDetector()

        val result = detector.update(
            accelX = 3.0f,
            accelY = 2.4f,
            accelZ = 10.6f,
            gyroX = 0.55f,
            gyroY = 0.35f,
            gyroZ = 0.48f
        )

        assertFalse(result.detected)
    }

    @Test
    fun `ignores quick phone turn without impact`() {
        val detector = PotholeDetector()

        val result = detector.update(
            accelX = 1.2f,
            accelY = 0.8f,
            accelZ = 9.9f,
            gyroX = 1.3f,
            gyroY = 1.1f,
            gyroZ = 0.7f
        )

        assertFalse(result.detected)
        assertEquals("None", result.severity)
    }

    @Test
    fun `does not mark a single strong hit as severe`() {
        val detector = PotholeDetector()

        val result = detector.update(
            accelX = 0f,
            accelY = 0f,
            accelZ = 26.0f,
            gyroX = 4.2f,
            gyroY = 3.6f,
            gyroZ = 1.2f
        )

        assertFalse(result.severity == "Severe")
    }

    @Test
    fun `confidence varies with signal strength`() {
        val lightDetector = PotholeDetector()
        val lightResults = listOf(
            Triple(0f, 0f, 15.2f) to Triple(2.6f, 0.6f, 0.2f),
            Triple(0f, 0f, 15.4f) to Triple(2.7f, 0.6f, 0.2f),
            Triple(0f, 0f, 15.6f) to Triple(2.8f, 0.7f, 0.2f)
        ).map { (accel, gyro) ->
            lightDetector.update(accel.first, accel.second, accel.third, gyro.first, gyro.second, gyro.third)
        }

        val strongDetector = PotholeDetector()
        val strongResults = listOf(
            Triple(0f, 0f, 17.0f) to Triple(3.0f, 1.4f, 0.5f),
            Triple(0f, 0f, 19.0f) to Triple(3.4f, 1.7f, 0.6f),
            Triple(0f, 0f, 22.0f) to Triple(3.8f, 1.9f, 0.8f)
        ).map { (accel, gyro) ->
            strongDetector.update(accel.first, accel.second, accel.third, gyro.first, gyro.second, gyro.third)
        }

        val light = lightResults.last()
        val strong = strongResults.last()

        assertTrue(light.detected)
        assertTrue(strong.detected)
        assertTrue(strong.confidence > light.confidence)
        assertNotEquals(72, light.confidence)
        assertNotEquals(96, strong.confidence)
    }
}

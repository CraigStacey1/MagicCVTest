// Copyright (c) 2024 Magic Tech Ltd

package fit.magic.cv.repcounter

import android.annotation.SuppressLint
import com.google.mediapipe.tasks.components.containers.NormalizedLandmark
import fit.magic.cv.PoseLandmarkerHelper
import kotlin.math.sqrt
import kotlin.math.pow
import kotlin.math.acos
import kotlin.math.sin
import kotlin.math.cos
import kotlin.math.PI
import kotlin.math.max
import kotlin.math.min

data class Vector(val x: Double, val y: Double, val z: Double)
data class Keypoint(val x: Double, val y: Double, val z: Double)
class ExerciseRepCounterImpl : ExerciseRepCounter() {

    private var previousFrame  = emptyList<Keypoint>()
    private var halfRep = false
    private var isFirst = true
    private var idealAngle = 95.0
    private var maxAngle = 150.0
    private var frameCount = 20

    private var currentOrientation: Double = 0.0
    private var movingAverage = MovingAverage(frameCount)



    override fun setResults(resultBundle: PoseLandmarkerHelper.ResultBundle) {
        // process pose data in resultBundle
        //
        // use functions in base class incrementRepCount(), sendProgressUpdate(),
        // and sendFeedbackMessage() to update the UI

        val poseKalmanFilter = PoseKalmanFilter()
        val orientationMovingAverage = MovingAverage(frameCount)

        poseKalmanFilter.initializeFilters(33)

        if (resultBundle.results[0].landmarks().isNotEmpty()) {
            var poseKeypoints = preprocessKeypoints(resultBundle.results[0].landmarks()[0])

            orientationMovingAverage.add(getOrientation(poseKeypoints))

            val orientation: Double =orientationMovingAverage.getAverage()

            if(currentOrientation-18 > orientation || orientation > currentOrientation+18) {
                currentOrientation = orientation
                isFirst = true
                previousFrame = emptyList()
                movingAverage.clear()
                orientationMovingAverage.clear()
            }

            if (isFirst) {
                currentOrientation = orientation
                previousFrame = poseKeypoints
                isFirst = false
            }

            poseKeypoints = poseKalmanFilter.update(poseKeypoints)

            if (poseKeypoints.isNotEmpty()) {
                if(-18 < currentOrientation && currentOrientation < 18) {
                    idealAngle = 50.0
                    updateFrontProgress(poseKeypoints)
                }
                else{
                    idealAngle = 90.0
                    updateProgress(poseKeypoints)
                }
            }
            previousFrame = poseKeypoints
        }
        else
            if(previousFrame.isNotEmpty()) {
                currentOrientation = 0.0
                isFirst = true
                previousFrame = emptyList()
                movingAverage.clear()
            }

    }

    private fun updateFrontProgress(poseKeypoints: List<Keypoint>){
        val angleKneeLeft = getAngle(poseKeypoints[23], poseKeypoints[25], poseKeypoints[27])
        val angleKneeRight = getAngle(poseKeypoints[24], poseKeypoints[26], poseKeypoints[28])

        val completionKneeLeft =((maxAngle - angleKneeLeft)/ (maxAngle - idealAngle))
        val completionKneeRight =((maxAngle - angleKneeRight)/ (maxAngle - idealAngle))

        var completionLeft = completionKneeLeft
        var completionRight = completionKneeRight

        if(completionLeft < 0.0) completionLeft = 0.0
        if(completionLeft > 1.0) completionLeft = 1.0

        if(completionRight < 0.0) completionRight = 0.0
        if(completionRight > 1.0) completionRight = 1.0

        val completion = max((completionLeft), (completionRight))

        movingAverage.add(completion)

        var percentage: Double = 0.0

        if(movingAverage.getSize() == frameCount) {
            if(movingAverage.getAverage() < 0.9) {

                val kalmanFilter = PercentageKalmanFilter()

                kalmanFilter.initializeFilters(frameCount)

                val postKalman = kalmanFilter.update(movingAverage.getList())

                percentage = postKalman.sum() / postKalman.size
            }
            else{
                percentage = movingAverage.getAverage()
            }
        }
        else{
            percentage = movingAverage.getAverage()
        }

        if(percentage >= 0.99){
            halfRep = true
        }
        if(percentage <= 0.1 && halfRep)
        {
            incrementRepCount()
            halfRep = false
        }
        sendProgressUpdate(percentage.toFloat())
    }

    @SuppressLint("DefaultLocale")
    private fun updateProgress(poseKeypoints : List<Keypoint>){
        val angleKneeLeft = getAngle(poseKeypoints[23], poseKeypoints[25], poseKeypoints[27])
        val angleKneeRight = getAngle(poseKeypoints[24], poseKeypoints[26], poseKeypoints[28])
        val angleKneeToAnkleRight = getAngle(poseKeypoints[28], poseKeypoints[26], poseKeypoints[27])
        val angleKneeToAnkleLeft = getAngle(poseKeypoints[28], poseKeypoints[25], poseKeypoints[27])
        val angleHipLeft = getAngle(poseKeypoints[25], poseKeypoints[23], poseKeypoints[11])
        val angleHipRight = getAngle(poseKeypoints[26], poseKeypoints[24], poseKeypoints[12])

        var completionLeft =((maxAngle - angleKneeLeft)/ (maxAngle - idealAngle))
        var completionRight =((maxAngle - angleKneeRight)/ (maxAngle - idealAngle))
        var completionHipLeft =((maxAngle - angleHipLeft)/ (maxAngle - idealAngle))
        var completionHipRight =((maxAngle - angleHipRight)/ (maxAngle - idealAngle))
        var completionKneeToAnkle = 1 - ((maxAngle - max(angleKneeToAnkleRight, angleKneeToAnkleLeft))/ (maxAngle - idealAngle))

        if(completionLeft < 0.0) completionLeft = 0.0
        if(completionLeft > 1.0) completionLeft = 1.0

        if(completionRight < 0.0) completionRight = 0.0
        if(completionRight > 1.0) completionRight = 1.0

        if(completionKneeToAnkle < 0.0) completionKneeToAnkle = 0.0
        if(completionKneeToAnkle > 1.0) completionKneeToAnkle = 1.0

        var completion = 0.0
        if(completionKneeToAnkle < 0.4)
            completion = max(completionHipLeft * 0.3, completionHipRight * 0.3) + max((completionLeft * 0.3), (completionRight * 0.3)) + (completionKneeToAnkle * 0.4)
        else
            completion = min(completionLeft, completionRight)

        movingAverage.add(completion)

        var percentage: Double = 0.0

        if(movingAverage.getSize() == frameCount) {

            if(movingAverage.getAverage() < 0.9) {

                val kalmanFilter = PercentageKalmanFilter()

                kalmanFilter.initializeFilters(frameCount)

                val postKalman = kalmanFilter.update(movingAverage.getList())

                percentage = postKalman.sum() / postKalman.size
            }
            else{
                percentage = movingAverage.getAverage()
            }
        }
        else {
            percentage = movingAverage.getAverage()
        }

        if(percentage >= 0.99){
            halfRep = true
        }
        if(percentage <= 0.1 && halfRep)
        {
            incrementRepCount()
            halfRep = false
        }

        sendProgressUpdate(percentage.toFloat())
    }

    private fun preprocessKeypoints(landmarks: List<NormalizedLandmark>): List<Keypoint>{
        if(landmarks.isEmpty()) return emptyList()

        return landmarks.map{ Keypoint(it.x().toDouble(), it.y().toDouble(), it.z().toDouble())}
    }

    private fun getAngle(firstKeypoint: Keypoint, centerKeypoint: Keypoint, thirdKeypoint: Keypoint): Double{
        val firstToCenter:Double = sqrt(
            (firstKeypoint.x  - centerKeypoint.x ).toDouble().pow(2.0) +
                    (firstKeypoint.y  - centerKeypoint.y ).toDouble().pow(2.0)
        )
        val firstToThird:Double  = sqrt(
            ((firstKeypoint.x  - thirdKeypoint.x ).toDouble()).pow(2.0) +
                    (firstKeypoint.y  - thirdKeypoint.y ).toDouble().pow(2.0)
        )
        val centerToThird:Double  = sqrt(
            ((centerKeypoint.x  - thirdKeypoint.x ).toDouble()).pow(2.0) +
                    (centerKeypoint.y  - thirdKeypoint.y ).toDouble().pow(2.0)
        )

        val ratio = (firstToCenter*firstToCenter + centerToThird*centerToThird - firstToThird*firstToThird)/ (2 * centerToThird * firstToCenter)
        val angle = acos(ratio)*(180/PI)

        return angle
    }

    private fun getOrientation(keypointFrame: List<Keypoint>): Double{
        val shoulderVector = Vector(
            (keypointFrame[11].x - keypointFrame[12].x),
            (keypointFrame[11].y  - keypointFrame[12].y),
            (keypointFrame[11].z  - keypointFrame[12].z) ,
        )

        val hipVector = Vector(
            (keypointFrame[23].x  - keypointFrame[24].x) ,
            (keypointFrame[23].y  - keypointFrame[24].y) ,
            (keypointFrame[23].z  - keypointFrame[24].z) ,
        )

        val referenceVector = Vector(1.0, 0.0, 0.0)

        val shoulderOrientation = getVectorOrientation(shoulderVector, referenceVector)
        val hipOrientation = getVectorOrientation(hipVector, referenceVector)

        return (shoulderOrientation + hipOrientation) / 2
    }

    private fun getVectorOrientation(left: Vector, right: Vector):Double{
        val dot: Double = (left.x * right.x + left.y * right.y + left.z * right.z).toDouble()
        val magnitudeLeft: Double = sqrt(left.x * left.x + left.y * left.y + left.z * left.z).toDouble()
        val magnitudeRight: Double  = sqrt(right.x * right.x + right.y * right.y + right.z * right.z).toDouble()

        return acos(dot / (magnitudeLeft*magnitudeRight))*(180/PI)
    }
}
class MovingAverage(private val windowSize: Int) {
    private var values = mutableListOf<Double>()
    private var sum = 0.0

    fun add(value: Double): Double {
        if (values.size == windowSize) {
            // Remove the oldest value from the sum and the list
            sum -= values.removeAt(0)
        }
        // Add the new value
        values.add(value)
        sum += value
        // Return the current moving average
        return getAverage()
    }

    fun getAverage(): Double {
        return if (values.isNotEmpty()) sum / values.size else 0.0
    }

    fun getSize():Int {
        return values.size
    }
    fun getList(): MutableList<Double>{
        return values
    }

    fun setList(newValues: List<Double>)
    {
        values = newValues.toMutableList()
    }

    fun clear() {
        values.clear()
        sum = 0.0
    }
}

// Class to represent the Kalman filter
class KalmanFilter(
    private var processNoise: Double  = 1e-5, // Process noise covariance
    private var measurementNoise: Double  = 1e-1, // Measurement noise covariance
    private var errorCovariance: Double  = 1.0 // Initial error covariance
) {
    private var stateEstimate: Double  = 0.0 // Initial state estimate

    fun update(measurement: Double ): Double  {
        // Kalman Gain
        val kalmanGain = errorCovariance / (errorCovariance + measurementNoise)

        // Update state estimate
        stateEstimate += kalmanGain * (measurement - stateEstimate)

        // Update error covariance
        errorCovariance = (1 - kalmanGain) * errorCovariance + processNoise

        return stateEstimate
    }
}

// Kalman filter wrapper for 3D pose points
class PoseKalmanFilter {
    private val filtersX = mutableListOf<KalmanFilter>()
    private val filtersY = mutableListOf<KalmanFilter>()
    private val filtersZ = mutableListOf<KalmanFilter>()

    fun initializeFilters(numPoints: Int, processNoise: Double  = 1e-5, measurementNoise: Double  = 1e-1) {
        for (i in 0 until numPoints) {
            filtersX.add(KalmanFilter(processNoise, measurementNoise))
            filtersY.add(KalmanFilter(processNoise, measurementNoise))
            filtersZ.add(KalmanFilter(processNoise, measurementNoise))
        }
    }

    fun update(points: List<Keypoint>): List<Keypoint> {
        if (points.size != filtersX.size) {
            throw IllegalArgumentException("Number of points must match the number of initialized filters")
        }

        return points.mapIndexed { index, point ->
            val smoothedX = filtersX[index].update(point.x)
            val smoothedY = filtersY[index].update(point.y)
            val smoothedZ = filtersZ[index].update(point.z)
            Keypoint(smoothedX, smoothedY, smoothedZ)
        }
    }
}

// Kalman filter wrapper for 3D pose points
class PercentageKalmanFilter {
    private val filters = mutableListOf<KalmanFilter>()

    fun initializeFilters(numPoints: Int, processNoise: Double  = 1e-5, measurementNoise: Double  = 1e-1) {
        for (i in 0 until numPoints) {
            filters.add(KalmanFilter(processNoise, measurementNoise))
        }
    }

    fun update(points: List<Double>): List<Double>{
        if (points.size != filters.size) {
            throw IllegalArgumentException("Number of points must match the number of initialized filters")
        }

        return points.mapIndexed { index, point ->
            val smoothed = filters[index].update(point)
            smoothed
        }
    }
}

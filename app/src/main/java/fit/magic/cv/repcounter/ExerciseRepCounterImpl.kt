// Copyright (c) 2024 Magic Tech Ltd

package fit.magic.cv.repcounter

import android.annotation.SuppressLint
import com.google.mediapipe.tasks.components.containers.NormalizedLandmark
import fit.magic.cv.PoseLandmarkerHelper
import kotlin.math.sqrt
import kotlin.math.pow
import kotlin.math.acos
import kotlin.math.PI
import kotlin.math.abs
import kotlin.math.max
import kotlin.math.min

data class Vector(val x: Double, val y: Double, val z: Double)
data class Keypoint(val x: Double, val y: Double, val z: Double)
class ExerciseRepCounterImpl : ExerciseRepCounter() {

    private var previousFrame  = emptyList<Keypoint>()
    private var halfRep = false
    private var isFirst = true
    private var idealAngle = 100.0
    private var maxAngle = 150.0
    private var maxWarriorTorsoAngle = 170
    private var maxWarriorKneeAngle = 45
    private var maxWarriorLegsAngle = 5
    private var idealWarriorAngle = 105
    private var minWarriorLegAngle = 170

    private var idealVerticalHead = 90.0
    private var lowestVerticalHead = 80.0
    private var highestVerticalHead = 100.0




    private var frameCount = 10

    private var glitchDetection = false
    private var SecondAssignment = true

    private var currentOrientation: Double = 0.0
    private var movingAverage = MovingAverage(frameCount)
    private var movingAverageWarrior = MovingAverage(frameCount)
    private val poseKalmanFilter = PoseKalmanFilter()
    private val kalmanFilter = PercentageKalmanFilter()

    private val frameMovingAverage = PoseMovingAverage(20 , 1.5f)

    private val keypointFilter = KeypointFilter()

    private var previousTime = 0L

    override fun setResults(resultBundle: PoseLandmarkerHelper.ResultBundle) {
        // process pose data in resultBundle
        //
        // use functions in base class incrementRepCount(), sendProgressUpdate(),
        // and sendFeedbackMessage() to update the UI

        val currentTime = System.nanoTime()
        val timeDelta = (currentTime - previousTime)

        //sendFeedbackMessage("CurrentTime: $currentTime\r\nPreviousTime: $previousTime\r\nDeltaTime: $timeDelta")

        val orientationMovingAverage = MovingAverage(frameCount)

        if (resultBundle.results[0].landmarks().isNotEmpty()) {
            var poseKeypoints = preprocessKeypoints(resultBundle.results[0].landmarks()[0])

            orientationMovingAverage.add(getOrientation(poseKeypoints))


            val orientation: Double =orientationMovingAverage.getAverage()

            if(currentOrientation-18 > orientation || orientation > currentOrientation+18) {
                currentOrientation = orientation
                isFirst = true
                previousFrame = emptyList()
                movingAverage.clear()
                poseKalmanFilter.deInitializeFilters()
                kalmanFilter.deInitializeFilters()
                orientationMovingAverage.clear()
            }

            if (isFirst) {
                currentOrientation = orientation
                previousFrame = poseKeypoints
                isFirst = false
                poseKalmanFilter.initializeFilters(33, processNoise = 0.001, measurementNoise = 0.01)
                kalmanFilter.initializeFilters(frameCount, processNoise = 0.001, measurementNoise = 0.001)
            }
            poseKeypoints = poseKalmanFilter.update(poseKeypoints)

            poseKeypoints = frameMovingAverage.add(poseKeypoints)

            if(glitchDetection){
                poseKeypoints = keypointFilter.filterKeypoints(poseKeypoints, timeDelta)
            }

            if (poseKeypoints.isNotEmpty()) {
                if(SecondAssignment){
                    updateWarriorProgress(poseKeypoints)
                }
                else if(-18 < currentOrientation && currentOrientation < 18) {
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
                poseKalmanFilter.deInitializeFilters()
                kalmanFilter.deInitializeFilters()
                glitchDetection = false
            }

        previousTime = currentTime

    }
    private fun updateWarriorProgress(poseKeypoints: List<Keypoint>) {
        val angleTorso = getAngle(poseKeypoints[27], poseKeypoints[23], poseKeypoints[11])
        val angleLegs = getAngle(poseKeypoints[27], poseKeypoints[23], poseKeypoints[28])

        val leftArm = getAngle(poseKeypoints[11], poseKeypoints[13], poseKeypoints[15])
        val rightArm= getAngle(poseKeypoints[12], poseKeypoints[14], poseKeypoints[16])

        val leftArmToTorso = getAngle(poseKeypoints[13], poseKeypoints[11], poseKeypoints[23])
        val rightArmToTorso = getAngle(poseKeypoints[14], poseKeypoints[12], poseKeypoints[24])

        val armsAngle = (leftArm+rightArm)/2
        val armsToTorsoAngle = (leftArmToTorso+rightArmToTorso)/2

        val headPosition = getHeadOrientation(poseKeypoints)
        var message = ""
        if (armsAngle > 150 && armsToTorsoAngle < 20) {
            message = "Arms Back"
            idealWarriorAngle = 110 //loosten contraints to account for obscured keypoints
        }
        if (armsAngle > 150 && armsToTorsoAngle > 150)
            message = "arms forward"
        if (60 > armsAngle && armsAngle > 40)
            message = "arms front"


        //sendFeedbackMessage("angleTorso: $angleTorso\r\nangleLegs : $angleLegs\r\nVariation: $message")

        val completionTorso =((maxWarriorTorsoAngle - angleTorso)/ (maxWarriorTorsoAngle - idealWarriorAngle))
        val completionLegs =((maxWarriorLegsAngle - angleLegs)/ (maxWarriorLegsAngle - idealWarriorAngle))

        var completion = 0.0
        completion = completionLegs * 0.5 + completionTorso * 0.5

        if(completion > 0.95) {
            glitchDetection = true
            val leftLeg = getAngle(poseKeypoints[27], poseKeypoints[25], poseKeypoints[23])
            val rightLeg = getAngle(poseKeypoints[28], poseKeypoints[26], poseKeypoints[24])
            var completionHead = 0.0
            val verticalHeadPosition = headPosition.second
            if(verticalHeadPosition > idealVerticalHead) {
                completionHead =
                    ((highestVerticalHead - verticalHeadPosition) / (highestVerticalHead - idealVerticalHead))
            }
            if(verticalHeadPosition > idealVerticalHead){
                completionHead =
                    ((verticalHeadPosition - lowestVerticalHead) / (idealVerticalHead - lowestVerticalHead))
            }

            val completionLeftLeg =
                ((maxWarriorKneeAngle - leftLeg) / (maxWarriorKneeAngle - minWarriorLegAngle))
            val completionRightLeg =
                ((maxWarriorKneeAngle - rightLeg) / (maxWarriorKneeAngle - minWarriorLegAngle))

            completion =
                completionLegs * 0.3 + completionTorso * 0.3 + completionLeftLeg * 0.15 + completionRightLeg * 0.15 + completionHead * 0.1

            sendFeedbackMessage("Variation: $message")
        }
        else{
            glitchDetection = false
        }

        //sendFeedbackMessage("horizontal: ${headPosition.first}\r\nVertical: ${headPosition.second}")
        movingAverageWarrior.add(completion)

        var percentage = 0.0

        if(movingAverageWarrior.getSize() == frameCount) {
            if(movingAverageWarrior.getAverage() < 0.9) {

                val postKalman = kalmanFilter.update(movingAverageWarrior.getList())

                percentage = postKalman.sum() / postKalman.size
                percentage = movingAverageWarrior.getAverage()
            }
            else{
                percentage = movingAverageWarrior.getAverage()
            }
        }
        else{
            percentage = movingAverageWarrior.getAverage()
        }

        sendProgressUpdate(percentage.toFloat())
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

        var percentage = 0.0

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

    private fun getHeadOrientation(keypointFrame: List<Keypoint>): Pair<Double, Double>{
        val noseToRightEar = Vector(
            (keypointFrame[0].x - keypointFrame[8].x),
            (keypointFrame[0].y  - keypointFrame[8].y),
            (keypointFrame[0].z  - keypointFrame[8].z) ,
        )

        val noseToLeftEar = Vector(
            (keypointFrame[0].x  - keypointFrame[7].x) ,
            (keypointFrame[0].y  - keypointFrame[7].y) ,
            (keypointFrame[0].z  - keypointFrame[7].z) ,
        )
        val earToEar = Vector(
            (keypointFrame[7].x  - keypointFrame[8].x) ,
            (keypointFrame[7].y  - keypointFrame[8].y) ,
            (keypointFrame[7].z  - keypointFrame[8].z) ,
        )

        val earMidpoint = Vector(
            (keypointFrame[7].x + keypointFrame[8].x) / 2,
            (keypointFrame[7].y  + keypointFrame[8].y) / 2,
            (keypointFrame[7].z  + keypointFrame[8].z)  / 2,
        )

        val noseToEarMidpoint = Vector(
            (keypointFrame[0].x  - earMidpoint.x) ,
            (keypointFrame[0].y  - earMidpoint.y) ,
            (keypointFrame[0].z  - earMidpoint.z) ,
        )

        val horizontal = getVectorOrientation(earToEar, noseToEarMidpoint)
        //val noseToLeftEarOrientation = getVectorOrientation(noseToLeftEar, earToEar)

        val horizontalProjection = Vector(
            x = 0.0,
            y = 1.0,
            z = 0.0
        )

        val vertical = getVectorOrientation(earToEar, horizontalProjection)

        return Pair(horizontal, vertical)
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

class PoseMovingAverage(private val windowSize: Int, private val threshold: Float) {
    private val frames: ArrayDeque<List<Keypoint>> = ArrayDeque()

    fun add(newFrame: List<Keypoint>): List<Keypoint> {
        if (frames.isNotEmpty() && newFrame.size != frames.first().size) {
            throw IllegalArgumentException("Frame size must match the number of keypoints.")
        }

        // Add the new frame to the queue
        frames.addLast(newFrame)

        // Remove the oldest frame if the queue exceeds the window size
        if (frames.size > windowSize) {
            frames.removeFirst()
        }

        val movingAverage = get()

        // Filter out glitches based on the deviation threshold
        return newFrame.mapIndexed { index, keypoint ->
            val avg = movingAverage[index]
            if (isGlitch(keypoint, avg)) {
                avg // Replace with the moving average if it's a glitch
            } else {
                keypoint // Keep the original value if it's within the threshold
            }
        }
    }

    private fun get(): List<Keypoint> {
        val numKeypoints = frames.first().size
        val averages = MutableList(numKeypoints) { Keypoint(0.0, 0.0, 0.0) }

        for (frame in frames) {
            for ((index, keypoint) in frame.withIndex()) {
                val avg = averages[index]
                averages[index] = Keypoint(
                    x = avg.x + keypoint.x,
                    y = avg.y + keypoint.y,
                    z = avg.z + keypoint.z
                )
            }
        }

        return averages.map { keypoint ->
            Keypoint(
                x = keypoint.x / frames.size,
                y = keypoint.y / frames.size,
                z = keypoint.z / frames.size
            )
        }
    }
    private fun isGlitch(keypoint: Keypoint, average: Keypoint): Boolean {
        val deviationX = abs(keypoint.x - average.x)
        val deviationY = abs(keypoint.y - average.y)
        val deviationZ = abs(keypoint.z - average.z)

        return deviationX > threshold || deviationY > threshold || deviationZ > threshold
    }
}

// Class to represent the Kalman filter
class KalmanFilter(
    private var processNoise: Double  = 0.5, // Process noise covariance
    private var measurementNoise: Double  = 1e-10, // Measurement noise covariance
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

    fun initializeFilters(numPoints: Int, processNoise: Double  = 0.5, measurementNoise: Double  = 0.00006) {
        for (i in 0 until numPoints) {
            filtersX.add(KalmanFilter(processNoise, measurementNoise))
            filtersY.add(KalmanFilter(processNoise, measurementNoise))
            filtersZ.add(KalmanFilter(processNoise, measurementNoise))
        }
    }

    fun deInitializeFilters() {
            filtersX.clear()
            filtersY.clear()
            filtersZ.clear()
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

    fun initializeFilters(numPoints: Int, processNoise: Double  = 0.5, measurementNoise: Double  = 0.00006) {
        for (i in 0 until numPoints) {
            filters.add(KalmanFilter(processNoise, measurementNoise))
        }
    }

    fun deInitializeFilters() {
        filters.clear()
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

class KeypointFilter(
    private val smoothingFactor: Double = 0.9,
    private val velocityThreshold: Double = 1.0e-9,
    private val displacementThreshold: Double = 0.02
){
    private val previousKeypoints = mutableListOf<Keypoint>()
    private val velocities = mutableListOf<Triple<Double, Double, Double>>()

    /**
     * Filters a list of keypoints using velocity thresholds and exponential smoothing.
     */
    fun filterKeypoints(currentKeypoints: List<Keypoint>, deltaTime: Long): List<Keypoint> {

        var keypoints = currentKeypoints.toMutableList()
        val legKeypoints = currentKeypoints.subList(23, 33)
        if (previousKeypoints.isEmpty()) {
            // Initialize the previousKeypoints and velocities on the first frame
            previousKeypoints.addAll(legKeypoints)
            velocities.addAll(legKeypoints.map { Triple(0.0, 0.0, 0.0) })

            keypoints.subList(23, keypoints.size).clear()
            keypoints.addAll(legKeypoints)

            return keypoints.toList()
        }
        val filteredKeypoints = mutableListOf<Keypoint>()

        for (i in legKeypoints.indices) {
            val current = legKeypoints[i]
            val previous = previousKeypoints[i]

            // Calculate velocity for x, y, and z
            val velocityX = (current.x - previous.x) / deltaTime
            val velocityY = (current.y - previous.y) / deltaTime
            val velocityZ = (current.z - previous.z) / deltaTime

            // Calculate total displacement
            val displacement = sqrt(
                (current.x - previous.x).pow(2.0) +
                        (current.y - previous.y).pow(2.0) +
                        (current.z - previous.z).pow(2.0)
            )

            //println("velocityX: " + abs(maxvelocityX) + " velocityY: " + abs(maxvelocityY)+" velocityZ: " + abs(maxvelocityZ))
            //println("displacement: $displacement maxDisplacemet: $maxDisplacemet")

            //println("\r\n")

            // Check if the velocity exceeds the threshold
            if (abs(velocityX) > velocityThreshold ||
                abs(velocityY) > velocityThreshold ||
                abs(velocityZ) > velocityThreshold ||
                displacement > displacementThreshold) {
                // Use exponential smoothing to stabilize the keypoint
                val smoothedX = smoothingFactor * previous.x + (1 - smoothingFactor) * current.x
                val smoothedY = smoothingFactor * previous.y + (1 - smoothingFactor) * current.y
                val smoothedZ = smoothingFactor * previous.z + (1 - smoothingFactor) * current.z
                //filteredKeypoints.add(Keypoint(smoothedX, smoothedY, smoothedZ))
                filteredKeypoints.add(previous)
            } else {
                filteredKeypoints.add(current)
            }

            // Update velocities
            velocities[i] = Triple(velocityX, velocityY, velocityZ)
        }


        keypoints.subList(23, keypoints.size).clear()
        keypoints.addAll(filteredKeypoints)
        // Update previous keypoints for the next frame
        previousKeypoints.clear()
        previousKeypoints.addAll(filteredKeypoints)


        return keypoints
    }
}

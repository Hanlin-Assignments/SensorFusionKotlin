package com.example.sensorfusion

import android.app.Activity
import android.content.Context
import android.graphics.Color
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Bundle
import android.os.Handler
import android.view.View
import android.widget.RadioGroup
import android.widget.TextView
import java.math.RoundingMode
import java.text.DecimalFormat
import java.util.*

import com.jjoe64.graphview.GraphView
import com.jjoe64.graphview.series.DataPoint
import com.jjoe64.graphview.series.LineGraphSeries


class MainActivity : Activity(), SensorEventListener, RadioGroup.OnCheckedChangeListener {
    private var mSensorManager: SensorManager? = null
    // angular speeds from gyro
    private val gyro = FloatArray(3)
    // rotation matrix from gyro data
    private var gyroMatrix = FloatArray(9)
    // orientation angles from gyro matrix
    private val gyroOrientation = FloatArray(3)
    // magnetic field vector
    private val magnet = FloatArray(3)
    // accelerometer vector
    private val accel = FloatArray(3)
    // orientation angles from accel and magnet
    private val accMagOrientation: FloatArray? = FloatArray(3)
    // final orientation angles from sensor fusion
    private val fusedOrientation = FloatArray(3)
    // accelerometer and magnetometer based rotation matrix
    private val rotationMatrix = FloatArray(9)
    private var timestamp = 0
    private var initState = true
    private val fuseTimer = Timer()
    // The following members are only for displaying the sensor output.
    var mHandler: Handler? = null
    private var mRadioGroup: RadioGroup? = null
    private var mAzimuthView: TextView? = null
    private var mPitchView: TextView? = null
    private var mRollView: TextView? = null
    private var radioSelection = 0
    var d = DecimalFormat("#.##")
    private lateinit var graph: GraphView
    private val series_azimuth = LineGraphSeries<DataPoint>()
    private val series_pitch = LineGraphSeries<DataPoint>()
    private val series_roll = LineGraphSeries<DataPoint>()
    private var timeValue = 0.0
    public override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)
        gyroOrientation[0] = 0.0f
        gyroOrientation[1] = 0.0f
        gyroOrientation[2] = 0.0f
        // initialise gyroMatrix with identity matrix
        gyroMatrix[0] = 1.0f
        gyroMatrix[1] = 0.0f
        gyroMatrix[2] = 0.0f
        gyroMatrix[3] = 0.0f
        gyroMatrix[4] = 1.0f
        gyroMatrix[5] = 0.0f
        gyroMatrix[6] = 0.0f
        gyroMatrix[7] = 0.0f
        gyroMatrix[8] = 1.0f
        // get sensorManager and initialise sensor listeners
        mSensorManager = this.getSystemService(Context.SENSOR_SERVICE) as SensorManager
        initListeners()
        // wait for one second until gyroscope and magnetometer/accelerometer
        // data is initialised then schedule the complementary filter task
        fuseTimer.scheduleAtFixedRate(calculateFusedOrientationTask(),
                1000, TIME_CONSTANT.toLong())
        // GUI stuff
        mHandler = Handler()
        radioSelection = 0
        d.roundingMode = RoundingMode.HALF_UP
        d.maximumFractionDigits = 3
        d.minimumFractionDigits = 3
        mRadioGroup = findViewById<View>(R.id.radioGroup1) as RadioGroup
        mAzimuthView = findViewById<View>(R.id.textView4) as TextView
        mPitchView = findViewById<View>(R.id.textView5) as TextView
        mRollView = findViewById<View>(R.id.textView6) as TextView
        mRadioGroup!!.setOnCheckedChangeListener(this)
        graph = findViewById<View>(R.id.graph) as GraphView
        graph.viewport.isScrollable = true
    }

    public override fun onStop() {
        super.onStop()
        // unregister sensor listeners to prevent the activity from draining the device's battery.
        mSensorManager!!.unregisterListener(this)
    }

    override fun onPause() {
        super.onPause()
        // unregister sensor listeners to prevent the activity from draining the device's battery.
        mSensorManager!!.unregisterListener(this)
    }

    public override fun onResume() {
        super.onResume()
        // restore the sensor listeners when user resumes the application.
        initListeners()
    }

    // This function registers sensor listeners for the accelerometer, magnetometer and gyroscope.
    private fun initListeners() {
        mSensorManager!!.registerListener(this,
                mSensorManager!!.getDefaultSensor(Sensor.TYPE_ACCELEROMETER),
                SensorManager.SENSOR_DELAY_FASTEST)
        mSensorManager!!.registerListener(this,
                mSensorManager!!.getDefaultSensor(Sensor.TYPE_GYROSCOPE),
                SensorManager.SENSOR_DELAY_FASTEST)
        mSensorManager!!.registerListener(this,
                mSensorManager!!.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD),
                SensorManager.SENSOR_DELAY_FASTEST)
    }

    override fun onAccuracyChanged(sensor: Sensor, accuracy: Int) {}
    override fun onSensorChanged(event: SensorEvent) {
        when (event.sensor.type) {
            Sensor.TYPE_ACCELEROMETER -> {
                // copy new accelerometer data into accel array and calculate orientation
                System.arraycopy(event.values, 0, accel, 0, 3)
                calculateAccMagOrientation()
            }
            Sensor.TYPE_GYROSCOPE ->  // process gyro data
                gyroFunction(event)
            Sensor.TYPE_MAGNETIC_FIELD ->  // copy new magnetometer data into magnet array
                System.arraycopy(event.values, 0, magnet, 0, 3)
        }
    }

    // calculates orientation angles from accelerometer and magnetometer output
    private fun calculateAccMagOrientation() {
        if (SensorManager.getRotationMatrix(rotationMatrix, null, accel, magnet)) {
            SensorManager.getOrientation(rotationMatrix, accMagOrientation)
        }
    }

    // This function is borrowed from the Android reference
    // at http://developer.android.com/reference/android/hardware/SensorEvent.html#values
    // It calculates a rotation vector from the gyroscope angular speed values.
    private fun getRotationVectorFromGyro(gyroValues: FloatArray,
                                          deltaRotationVector: FloatArray,
                                          timeFactor: Float) {
        val normValues = FloatArray(3)
        // Calculate the angular speed of the sample
        val omegaMagnitude = Math.sqrt(gyroValues[0] * gyroValues[0] + gyroValues[1] * gyroValues[1] + (
                gyroValues[2] * gyroValues[2]).toDouble()).toFloat()
        // Normalize the rotation vector if it's big enough to get the axis
        if (omegaMagnitude > EPSILON) {
            normValues[0] = gyroValues[0] / omegaMagnitude
            normValues[1] = gyroValues[1] / omegaMagnitude
            normValues[2] = gyroValues[2] / omegaMagnitude
        }
        // Integrate around this axis with the angular speed by the timestamp
        // in order to get a delta rotation from this sample over the timestamp
        // We will convert this axis-angle representation of the delta rotation
        // into a quaternion before turning it into the rotation matrix.
        val thetaOverTwo = omegaMagnitude * timeFactor
        val sinThetaOverTwo = Math.sin(thetaOverTwo.toDouble()).toFloat()
        val cosThetaOverTwo = Math.cos(thetaOverTwo.toDouble()).toFloat()
        deltaRotationVector[0] = sinThetaOverTwo * normValues[0]
        deltaRotationVector[1] = sinThetaOverTwo * normValues[1]
        deltaRotationVector[2] = sinThetaOverTwo * normValues[2]
        deltaRotationVector[3] = cosThetaOverTwo
    }

    // This function performs the integration of the gyroscope data.
    // It writes the gyroscope based orientation into gyroOrientation.
    private fun gyroFunction(event: SensorEvent) { // don't start until first accelerometer/magnetometer orientation has been acquired
        if (accMagOrientation == null) return
        // initialisation of the gyroscope based rotation matrix
        if (initState) {
            var initMatrix = getRotationMatrixFromOrientation(accMagOrientation)
            val test = FloatArray(3)
            SensorManager.getOrientation(initMatrix, test)
            gyroMatrix = matrixMultiplication(gyroMatrix, initMatrix)
            initState = false
        }
        // copy the new gyro values into the gyro array
        // convert the raw gyro data into a rotation vector
        val deltaVector = FloatArray(4)
        if (timestamp != 0) {
            val dT = (event.timestamp - timestamp) * NS2S
            System.arraycopy(event.values, 0, gyro, 0, 3)
            getRotationVectorFromGyro(gyro, deltaVector, dT / 2.0f)
        }
        // measurement done, save current time for next interval
        timestamp = event.timestamp.toInt()
        // convert rotation vector into rotation matrix
        val deltaMatrix = FloatArray(9)
        SensorManager.getRotationMatrixFromVector(deltaMatrix, deltaVector)
        // apply the new rotation interval on the gyroscope based rotation matrix
        gyroMatrix = matrixMultiplication(gyroMatrix, deltaMatrix)
        // get the gyroscope based orientation from the rotation matrix
        SensorManager.getOrientation(gyroMatrix, gyroOrientation)
    }

    private fun getRotationMatrixFromOrientation(o: FloatArray): FloatArray {
        val xM = FloatArray(9)
        val yM = FloatArray(9)
        val zM = FloatArray(9)
        val sinX = Math.sin(o[1].toDouble()).toFloat()
        val cosX = Math.cos(o[1].toDouble()).toFloat()
        val sinY = Math.sin(o[2].toDouble()).toFloat()
        val cosY = Math.cos(o[2].toDouble()).toFloat()
        val sinZ = Math.sin(o[0].toDouble()).toFloat()
        val cosZ = Math.cos(o[0].toDouble()).toFloat()
        // rotation about x-axis (pitch)
        xM[0] = 1.0f
        xM[1] = 0.0f
        xM[2] = 0.0f
        xM[3] = 0.0f
        xM[4] = cosX
        xM[5] = sinX
        xM[6] = 0.0f
        xM[7] = -sinX
        xM[8] = cosX
        // rotation about y-axis (roll)
        yM[0] = cosY
        yM[1] = 0.0f
        yM[2] = sinY
        yM[3] = 0.0f
        yM[4] = 1.0f
        yM[5] = 0.0f
        yM[6] = -sinY
        yM[7] = 0.0f
        yM[8] = cosY
        // rotation about z-axis (azimuth)
        zM[0] = cosZ
        zM[1] = sinZ
        zM[2] = 0.0f
        zM[3] = -sinZ
        zM[4] = cosZ
        zM[5] = 0.0f
        zM[6] = 0.0f
        zM[7] = 0.0f
        zM[8] = 1.0f
        // rotation order is y, x, z (roll, pitch, azimuth)
        var resultMatrix = matrixMultiplication(xM, yM)
        resultMatrix = matrixMultiplication(zM, resultMatrix)
        return resultMatrix
    }

    private fun matrixMultiplication(A: FloatArray, B: FloatArray): FloatArray {
        val result = FloatArray(9)
        result[0] = A[0] * B[0] + A[1] * B[3] + A[2] * B[6]
        result[1] = A[0] * B[1] + A[1] * B[4] + A[2] * B[7]
        result[2] = A[0] * B[2] + A[1] * B[5] + A[2] * B[8]
        result[3] = A[3] * B[0] + A[4] * B[3] + A[5] * B[6]
        result[4] = A[3] * B[1] + A[4] * B[4] + A[5] * B[7]
        result[5] = A[3] * B[2] + A[4] * B[5] + A[5] * B[8]
        result[6] = A[6] * B[0] + A[7] * B[3] + A[8] * B[6]
        result[7] = A[6] * B[1] + A[7] * B[4] + A[8] * B[7]
        result[8] = A[6] * B[2] + A[7] * B[5] + A[8] * B[8]
        return result
    }

    internal inner class calculateFusedOrientationTask : TimerTask() {
        override fun run() {
            val oneMinusCoeff = 1.0f - FILTER_COEFFICIENT
            /*
             * Fix for 179 degree <--> -179 degree transition problem:
             * Check whether one of the two orientation angles (gyro or accMag) is negative while the other one is positive.
             * If so, add 360 degree (2 * math.PI) to the negative value, perform the sensor fusion, and remove the 360 degree from the result
             * if it is greater than 180�. This stabilizes the output in positive-to-negative-transition cases.
             */
            // azimuth
            if (gyroOrientation[0] < -0.5 * Math.PI && accMagOrientation!![0] > 0.0) {
                fusedOrientation[0] = (FILTER_COEFFICIENT * (gyroOrientation[0] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[0]).toFloat()
                fusedOrientation[0] -= if (fusedOrientation[0] > Math.PI) (2.0 * Math.PI).toFloat() else 0.toFloat()
            } else if (accMagOrientation!![0] < -0.5 * Math.PI && gyroOrientation[0] > 0.0) {
                fusedOrientation[0] = (FILTER_COEFFICIENT * gyroOrientation[0] + oneMinusCoeff * (accMagOrientation[0] + 2.0 * Math.PI)).toFloat()
                fusedOrientation[0] -= if (fusedOrientation[0] > Math.PI) (2.0 * Math.PI).toFloat() else 0.toFloat()
            } else {
                fusedOrientation[0] = FILTER_COEFFICIENT * gyroOrientation[0] + oneMinusCoeff * accMagOrientation[0]
            }
            // pitch
            if (gyroOrientation[1] < -0.5 * Math.PI && accMagOrientation[1] > 0.0) {
                fusedOrientation[1] = (FILTER_COEFFICIENT * (gyroOrientation[1] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[1]).toFloat()
                fusedOrientation[1] -= if (fusedOrientation[1] > Math.PI) (2.0 * Math.PI).toFloat() else 0.toFloat()
            } else if (accMagOrientation[1] < -0.5 * Math.PI && gyroOrientation[1] > 0.0) {
                fusedOrientation[1] = (FILTER_COEFFICIENT * gyroOrientation[1] + oneMinusCoeff * (accMagOrientation[1] + 2.0 * Math.PI)).toFloat()
                fusedOrientation[1] -= if (fusedOrientation[1] > Math.PI) (2.0 * Math.PI).toFloat() else 0.toFloat()
            } else {
                fusedOrientation[1] = FILTER_COEFFICIENT * gyroOrientation[1] + oneMinusCoeff * accMagOrientation[1]
            }
            // roll
            if (gyroOrientation[2] < -0.5 * Math.PI && accMagOrientation[2] > 0.0) {
                fusedOrientation[2] = (FILTER_COEFFICIENT * (gyroOrientation[2] + 2.0 * Math.PI) + oneMinusCoeff * accMagOrientation[2]).toFloat()
                fusedOrientation[2] -= if (fusedOrientation[2] > Math.PI) (2.0 * Math.PI).toFloat() else 0.toFloat()
            } else if (accMagOrientation[2] < -0.5 * Math.PI && gyroOrientation[2] > 0.0) {
                fusedOrientation[2] = (FILTER_COEFFICIENT * gyroOrientation[2] + oneMinusCoeff * (accMagOrientation[2] + 2.0 * Math.PI)).toFloat()
                fusedOrientation[2] -= if (fusedOrientation[2] > Math.PI) (2.0 * Math.PI).toFloat() else 0.toFloat()
            } else {
                fusedOrientation[2] = FILTER_COEFFICIENT * gyroOrientation[2] + oneMinusCoeff * accMagOrientation[2]
            }
            // overwrite gyro matrix and orientation with fused orientation
            // to compensate gyro drift
            gyroMatrix = getRotationMatrixFromOrientation(fusedOrientation)
            System.arraycopy(fusedOrientation, 0, gyroOrientation, 0, 3)
            // update sensor output in GUI
            mHandler!!.post(updateOrientationDisplayTask)
            mHandler!!.post(updateDrawTask)
        }
    }

    // **************************** GUI FUNCTIONS *********************************
    override fun onCheckedChanged(group: RadioGroup, checkedId: Int) {
        when (checkedId) {
            R.id.radio0 -> radioSelection = 0
            R.id.radio1 -> radioSelection = 1
            R.id.radio2 -> radioSelection = 2
        }
    }

    private fun updateOrientationDisplay() {
        when (radioSelection) {
            0 -> {
                mAzimuthView!!.text = d.format(accMagOrientation!![0] * 180 / Math.PI) + " rad/s"
                mPitchView!!.text = d.format(accMagOrientation[1] * 180 / Math.PI) + " rad/s"
                mRollView!!.text = d.format(accMagOrientation[2] * 180 / Math.PI) + " rad/s"
            }
            1 -> {
                mAzimuthView!!.text = d.format(gyroOrientation[0] * 180 / Math.PI) + " rad/s"
                mPitchView!!.text = d.format(gyroOrientation[1] * 180 / Math.PI) + " rad/s"
                mRollView!!.text = d.format(gyroOrientation[2] * 180 / Math.PI) + " rad/s"
            }
            2 -> {
                mAzimuthView!!.text = d.format(fusedOrientation[0] * 180 / Math.PI) + " rad/s"
                mPitchView!!.text = d.format(fusedOrientation[1] * 180 / Math.PI) + " rad/s"
                mRollView!!.text = d.format(fusedOrientation[2] * 180 / Math.PI) + " rad/s"
            }
        }
    }

    private val updateOrientationDisplayTask = Runnable { updateOrientationDisplay() }

    companion object {
        const val EPSILON = 0.000000001f
        private const val NS2S = 1.0f / 1000000000.0f
        const val TIME_CONSTANT = 30
        const val FILTER_COEFFICIENT = 0.98f
    }

    private fun draw(){
        var mAzimuthValue = 0.0
        var mPitchValue = 0.0
        var mRollValue = 0.0

        when (radioSelection) {
            0 -> {
                mAzimuthValue = accMagOrientation!![0] * 180 / Math.PI
                mPitchValue = accMagOrientation[1] * 180 / Math.PI
                mRollValue = accMagOrientation[2] * 180 / Math.PI
            }
            1 -> {
                mAzimuthValue = gyroOrientation!![0] * 180 / Math.PI
                mPitchValue = gyroOrientation[1] * 180 / Math.PI
                mRollValue = gyroOrientation[2] * 180 / Math.PI
            }
            2 -> {
                mAzimuthValue = fusedOrientation!![0] * 180 / Math.PI
                mPitchValue = fusedOrientation[1] * 180 / Math.PI
                mRollValue = fusedOrientation[2] * 180 / Math.PI
            }
        }

        series_azimuth.appendData( DataPoint(timeValue++, mAzimuthValue), true, 20)
        series_pitch.appendData( DataPoint(timeValue++, mPitchValue), true, 20)
        series_roll.appendData( DataPoint(timeValue++, mRollValue), true, 20)
        series_azimuth.setColor(Color.RED)
        series_pitch.setColor(Color.BLUE)
        series_roll.setColor(Color.GREEN)

        graph.addSeries(series_azimuth)
        graph.addSeries(series_pitch)
        graph.addSeries(series_roll)
    }

    private val updateDrawTask = Runnable { draw() }
}


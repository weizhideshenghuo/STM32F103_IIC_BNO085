#include "BNO085_app.h"

// ------------------------------------------------------------------------
// Configure Compile time options for the demo app

// Define this to produce DSF data for logging
// #define DSF_OUTPUT

// Define this to use HMD-appropriate configuration.
// #define CONFIGURE_HMD

// Used for debug
#define DEBUG_MODE
// ------------------------------------------------------------------------

#ifdef CONFIGURE_HMD
    // Enable GIRV prediction for 28ms with 100Hz sync
    #define GIRV_PRED_AMT FIX_Q(10, 0.028)             // prediction amt: 28ms
#else
    // Disable GIRV prediction
    #define GIRV_PRED_AMT FIX_Q(10, 0.0)               // prediction amt: 0
#endif

#define FIX_Q(n, x) ((int32_t)(x * (float)(1 << n)))
const float scaleDegToRad = 3.14159265358 / 180.0;

// --- Private data ---------------------------------------------------

sh2_ProductIds_t prodIds;

sh2_Hal_t *pSh2Hal = 0;

bool resetOccurred = false;

float q[4];
uint8_t IMU_data[8] = {0x01,0x02,0x03,0x04,0x05,0x06,0x07,0x08}; //用于存放IMU三轴的姿态角

// --- Private methods ----------------------------------------------

void change_to_int16(float BNO080_Pitch, float BNO080_Roll, float BNO080_Yaw)
{
	int16_t pitch_100 = (int16_t)(BNO080_Pitch * 100);
	int16_t roll_100 = (int16_t)(BNO080_Roll * 100);
	int16_t yaw_100 = (int16_t)(BNO080_Yaw * 100);
	IMU_data[0] = (uint8_t)(pitch_100 >> 8 & 0xFF); //高位
	IMU_data[1] = (uint8_t)(pitch_100 & 0xFF);
	IMU_data[2] = (uint8_t)(roll_100 >> 8 & 0xFF); //高位
	IMU_data[3] = (uint8_t)(roll_100 & 0xFF);
	IMU_data[4] = (uint8_t)(yaw_100 >> 8 & 0xFF); //高位
	IMU_data[5] = (uint8_t)(yaw_100 & 0xFF);
}

// Configure one sensor to produce periodic reports
static void startReports()
{
    static sh2_SensorConfig_t config;
    int status;
    int sensorId;
    static const int enabledSensors[] =
    {
        SH2_GAME_ROTATION_VECTOR,
        // SH2_RAW_ACCELEROMETER,
        // SH2_RAW_GYROSCOPE,
        // SH2_ROTATION_VECTOR,
        // SH2_GYRO_INTEGRATED_RV,
    };

    // These sensor options are disabled or not used in most cases
    config.changeSensitivityEnabled = false;
    config.wakeupEnabled = false;
    config.changeSensitivityRelative = false;
    config.alwaysOnEnabled = false;
    config.changeSensitivity = 0;
    config.batchInterval_us = 0;
    config.sensorSpecific = 0;

    // Select a report interval.
    config.reportInterval_us = 10000;  // microseconds (100Hz)
    // config.reportInterval_us = 2500;   // microseconds (400Hz)
    // config.reportInterval_us = 1000;   // microseconds (1000Hz)

    for (int n = 0; n < ARRAY_LEN(enabledSensors); n++)
    {
        // Configure the sensor hub to produce these reports
        sensorId = enabledSensors[n];
        status = sh2_setSensorConfig(sensorId, &config);
#ifdef DEBUG_MODE
        if (status != 0) {
            printf("Error while enabling sensor %d\r\n", sensorId);
        }
#endif
    }
    
}

// Handle non-sensor events from the sensor hub
static void eventHandler(void * cookie, sh2_AsyncEvent_t *pEvent)
{
    // If we see a reset, set a flag so that sensors will be reconfigured.
    if (pEvent->eventId == SH2_RESET) {
        resetOccurred = true;
    }
}

#ifdef DSF_OUTPUT
// Print headers for DSF format output
static void printDsfHeaders(void)
{
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, ANG_POS_GLOBAL[rijk]{quaternion}, ANG_POS_ACCURACY[x]{rad}\r\n",
           SH2_ROTATION_VECTOR);
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, GAME_ROTATION_VECTOR[rijk]{quaternion}\r\n",
           SH2_GAME_ROTATION_VECTOR);
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, RAW_ACCELEROMETER[xyz]{adc units}\r\n",
           SH2_RAW_ACCELEROMETER);
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, RAW_MAGNETOMETER[xyz]{adc units}\r\n",
           SH2_RAW_MAGNETOMETER);
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, RAW_GYROSCOPE[xyz]{adc units}\r\n",
           SH2_RAW_GYROSCOPE);
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, ACCELEROMETER[xyz]{m/s^2}\r\n",
           SH2_ACCELEROMETER);
    printf("+%d TIME[x]{s}, SAMPLE_ID[x]{samples}, MAG_FIELD[xyz]{uTesla}, STATUS[x]{enum}\r\n",
           SH2_MAGNETIC_FIELD_CALIBRATED);
    printf("+%d TIME[x]{s}, ANG_VEL_GYRO_RV[xyz]{rad/s}, ANG_POS_GYRO_RV[wxyz]{quaternion}\r\n",
           SH2_GYRO_INTEGRATED_RV);
}
#endif


#ifdef DSF_OUTPUT
// Print a sensor event as a DSF record
static void printDsf(const sh2_SensorEvent_t * event)
{
    float t, r, i, j, k, acc_rad;
    float angVelX, angVelY, angVelZ;
    static uint32_t lastSequence[SH2_MAX_SENSOR_ID+1];  // last sequence number for each sensor
    sh2_SensorValue_t value;

    // Convert event to value
    sh2_decodeSensorEvent(&value, event);
    
    // Compute new sample_id
    uint8_t deltaSeq = value.sequence - (lastSequence[value.sensorId] & 0xFF);
    lastSequence[value.sensorId] += deltaSeq;

    // Get time as float
    t = value.timestamp / 1000000.0;
    
    switch (value.sensorId) {
        case SH2_RAW_ACCELEROMETER:
            printf(".%d %0.6f, %d, %d, %d, %d\r\n",
                   SH2_RAW_ACCELEROMETER,
                   t,
                   lastSequence[value.sensorId],
                   value.un.rawAccelerometer.x,
                   value.un.rawAccelerometer.y,
                   value.un.rawAccelerometer.z);
            break;
        
        case SH2_RAW_MAGNETOMETER:
            printf(".%d %0.6f, %d, %d, %d, %d\r\n",
                   SH2_RAW_MAGNETOMETER,
                   t,
                   lastSequence[value.sensorId],
                   value.un.rawMagnetometer.x,
                   value.un.rawMagnetometer.y,
                   value.un.rawMagnetometer.z);
            break;
        
        case SH2_RAW_GYROSCOPE:
            printf(".%d %0.6f, %d, %d, %d, %d\r\n",
                   SH2_RAW_GYROSCOPE,
                   t,
                   lastSequence[value.sensorId],
                   value.un.rawGyroscope.x,
                   value.un.rawGyroscope.y,
                   value.un.rawGyroscope.z);
            break;

        case SH2_MAGNETIC_FIELD_CALIBRATED:
            printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f, %u\r\n",
                   SH2_MAGNETIC_FIELD_CALIBRATED,
                   t,
                   lastSequence[value.sensorId],
                   value.un.magneticField.x,
                   value.un.magneticField.y,
                   value.un.magneticField.z,
                   value.status & 0x3
                );
            break;
        
        case SH2_ACCELEROMETER:
            printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f\r\n",
                   SH2_ACCELEROMETER,
                   t,
                   lastSequence[value.sensorId],
                   value.un.accelerometer.x,
                   value.un.accelerometer.y,
                   value.un.accelerometer.z);
            break;
        
        case SH2_ROTATION_VECTOR:
            r = value.un.rotationVector.real;
            i = value.un.rotationVector.i;
            j = value.un.rotationVector.j;
            k = value.un.rotationVector.k;
            acc_rad = value.un.rotationVector.accuracy;
            printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f\r\n",
                   SH2_ROTATION_VECTOR,
                   t,
                   lastSequence[value.sensorId],
                   r, i, j, k,
                   acc_rad);
						/*q[0] = value.un.gameRotationVector.i;
						q[1] = value.un.gameRotationVector.j;
						q[2] = value.un.gameRotationVector.k;
						q[3] = value.un.gameRotationVector.real;
						Quaternion_Update(&q[0]);
						printf("Pitch:%0.2f Roll:%0.2f Yaw:%0.2f\r\n",
                   BNO080_Pitch, BNO080_Roll, BNO080_Yaw);*/
            break;
        
        case SH2_GAME_ROTATION_VECTOR:
            r = value.un.gameRotationVector.real;
            i = value.un.gameRotationVector.i;
            j = value.un.gameRotationVector.j;
            k = value.un.gameRotationVector.k;
            printf(".%d %0.6f, %d, %0.6f, %0.6f, %0.6f, %0.6f\r\n",
                   SH2_GAME_ROTATION_VECTOR,
                   t,
                   lastSequence[value.sensorId],
                   r, i, j, k);
						/*q[0] = value.un.gameRotationVector.i;
						q[1] = value.un.gameRotationVector.j;
						q[2] = value.un.gameRotationVector.k;
						q[3] = value.un.gameRotationVector.real;
						Quaternion_Update(&q[0]);
						printf("Pitch:%0.2f Roll:%0.2f Yaw:%0.2f\r\n",
                   BNO080_Pitch, BNO080_Roll, BNO080_Yaw);*/
            break;
            
        case SH2_GYRO_INTEGRATED_RV:
            angVelX = value.un.gyroIntegratedRV.angVelX;
            angVelY = value.un.gyroIntegratedRV.angVelY;
            angVelZ = value.un.gyroIntegratedRV.angVelZ;
            r = value.un.gyroIntegratedRV.real;
            i = value.un.gyroIntegratedRV.i;
            j = value.un.gyroIntegratedRV.j;
            k = value.un.gyroIntegratedRV.k;
            printf(".%d %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f, %0.6f\r\n",
                   SH2_GYRO_INTEGRATED_RV,
                   t,
                   angVelX, angVelY, angVelZ,
                   r, i, j, k);
            break;
        default:
            printf("Unknown sensor: %d\r\n", value.sensorId);
            break;
    }
}
#endif

static void delayUs(uint32_t t)
{
    uint32_t now_us = pSh2Hal->getTimeUs(pSh2Hal);
    uint32_t start_us = now_us;

    while (t > (now_us - start_us))
    {
        now_us = pSh2Hal->getTimeUs(pSh2Hal);
    }
}

#ifndef DSF_OUTPUT
// Read product ids with version info from sensor hub and print them
static void reportProdIds(void)
{
    int status;
    
    memset(&prodIds, 0, sizeof(prodIds));
    status = sh2_getProdIds(&prodIds);
	
#ifdef DEBUG_MODE	
    if (status < 0) {
        printf("Error from sh2_getProdIds.\r\n");
        return;
    }

    // Report the results
    for (int n = 0; n < prodIds.numEntries; n++) {
        printf("Part %d : Version %d.%d.%d Build %d\r\n",
               prodIds.entry[n].swPartNumber,
               prodIds.entry[n].swVersionMajor, prodIds.entry[n].swVersionMinor, 
               prodIds.entry[n].swVersionPatch, prodIds.entry[n].swBuildNumber);

        // Wait a bit so we don't overflow the console output.
        delayUs(10000);
    }
#endif
		
}
#endif

#ifndef DSF_OUTPUT
// Print a sensor event to the console
static void printEvent(const sh2_SensorEvent_t * event)
{
    int rc;
    sh2_SensorValue_t value;
    float scaleRadToDeg = 180.0 / 3.14159265358;
    float r, i, j, k, acc_deg, x, y, z;
    float t;
    static int skip = 0;

    rc = sh2_decodeSensorEvent(&value, event);
    if (rc != SH2_OK) {
        printf("Error decoding sensor event: %d\r\n", rc);
        return;
    }

    t = value.timestamp / 1000000.0;  // time in seconds.
    switch (value.sensorId) {
        case SH2_RAW_ACCELEROMETER:
            printf("%8.4f Raw acc: %d %d %d\r\n",
                   t,
                   value.un.rawAccelerometer.x,
                   value.un.rawAccelerometer.y,
                   value.un.rawAccelerometer.z);
            break;

        case SH2_ACCELEROMETER:
            printf("%8.4f Acc: %f %f %f\r\n",
                   t,
                   value.un.accelerometer.x,
                   value.un.accelerometer.y,
                   value.un.accelerometer.z);
            break;
            
        case SH2_RAW_GYROSCOPE:
            printf("%8.4f Raw gyro: x:%d y:%d z:%d temp:%d time_us:%d\r\n",
                   t,
                   value.un.rawGyroscope.x,
                   value.un.rawGyroscope.y,
                   value.un.rawGyroscope.z,
                   value.un.rawGyroscope.temperature,
                   value.un.rawGyroscope.timestamp);
            break;
            
        case SH2_ROTATION_VECTOR: //九轴姿态融合
            r = value.un.rotationVector.real;
            i = value.un.rotationVector.i;
            j = value.un.rotationVector.j;
            k = value.un.rotationVector.k;
            acc_deg = scaleRadToDeg * 
                value.un.rotationVector.accuracy;
            printf("%8.4f Rotation Vector: "
                   "r:%0.6f i:%0.6f j:%0.6f k:%0.6f (acc: %0.6f deg)\r\n",
                   t,
                   r, i, j, k, acc_deg);
						/*q[0] = value.un.gameRotationVector.i;
						q[1] = value.un.gameRotationVector.j;
						q[2] = value.un.gameRotationVector.k;
						q[3] = value.un.gameRotationVector.real;
						Quaternion_Update(&q[0]);
						printf("Pitch:%0.2f Roll:%0.2f Yaw:%0.2f\r\n",
                   BNO080_Pitch, BNO080_Roll, BNO080_Yaw);*/
            break;
        case SH2_GAME_ROTATION_VECTOR: //初始时YAW为0
            /*r = value.un.gameRotationVector.real;
            i = value.un.gameRotationVector.i;
            j = value.un.gameRotationVector.j;
            k = value.un.gameRotationVector.k;
            printf("%8.4f GRV: "
                   "r:%0.6f i:%0.6f j:%0.6f k:%0.6f\r\n",
                   t,
                   r, i, j, k);*/
						q[0] = value.un.gameRotationVector.i;
						q[1] = value.un.gameRotationVector.j;
						q[2] = value.un.gameRotationVector.k;
						q[3] = value.un.gameRotationVector.real;
						Quaternion_Update(&q[0]);
#ifdef DEBUG_MODE	
						printf("Pitch:%0.2f Roll:%0.2f Yaw:%0.2f\r\n",
                   BNO080_Pitch, BNO080_Roll, BNO080_Yaw);
#endif
						change_to_int16(BNO080_Pitch, BNO080_Roll, BNO080_Yaw);
            break;
        case SH2_GYROSCOPE_CALIBRATED:
            x = value.un.gyroscope.x;
            y = value.un.gyroscope.y;
            z = value.un.gyroscope.z;
            printf("%8.4f GYRO: "
                   "x:%0.6f y:%0.6f z:%0.6f\r\n",
                   t,
                   x, y, z);
            break;
        case SH2_GYROSCOPE_UNCALIBRATED:
            x = value.un.gyroscopeUncal.x;
            y = value.un.gyroscopeUncal.y;
            z = value.un.gyroscopeUncal.z;
            printf("%8.4f GYRO_UNCAL: "
                   "x:%0.6f y:%0.6f z:%0.6f\r\n",
                   t,
                   x, y, z);
            break;
        case SH2_GYRO_INTEGRATED_RV:
            // These come at 1kHz, too fast to print all of them.
            // So only print every 10th one
            skip++;
            if (skip == 10) {
                skip = 0;
                r = value.un.gyroIntegratedRV.real;
                i = value.un.gyroIntegratedRV.i;
                j = value.un.gyroIntegratedRV.j;
                k = value.un.gyroIntegratedRV.k;
                x = value.un.gyroIntegratedRV.angVelX;
                y = value.un.gyroIntegratedRV.angVelY;
                z = value.un.gyroIntegratedRV.angVelZ;
                printf("%8.4f Gyro Integrated RV: "
                       "r:%0.6f i:%0.6f j:%0.6f k:%0.6f x:%0.6f y:%0.6f z:%0.6f\r\n",
                       t,
                       r, i, j, k,
                       x, y, z);
            }
            break;
        default:
            printf("Unknown sensor: %d\r\n", value.sensorId);
            break;
    }
}
#endif

// Handle sensor events.
static void sensorHandler(void * cookie, sh2_SensorEvent_t *pEvent)
{
#ifdef DSF_OUTPUT
    printDsf(pEvent);
#else
    printEvent(pEvent);
#endif
}

// --- Public methods -------------------------------------------------

// Initialize demo. 
void BNO085_init(void)
{
    int status;

#ifdef DEBUG_MODE
    printf("BNO085 SH2 Demo(IIC).\r\n");
#endif

    // Create HAL instance
    pSh2Hal = sh2_hal_init();

    // Open SH2 interface (also registers non-sensor event handler.)
    status = sh2_open(pSh2Hal, eventHandler, NULL);
	
#ifdef DEBUG_MODE
    if (status != SH2_OK) {
        printf("Error, %d, from sh2_open.\r\n", status);
    }
#endif
		
    // Register sensor listener
    sh2_setSensorCallback(sensorHandler, NULL);

#ifdef DSF_OUTPUT
		// Print DSF file headers
		printDsfHeaders();
#else
		// Read and display BNO080 product ids
		reportProdIds();
#endif

    // resetOccurred would have been set earlier.
    // We can reset it since we are starting the sensor reports now.
    resetOccurred = false;

    // Start the flow of sensor reports
    startReports();
}

// This must be called periodically.  (The demo main calls it continuously in a loop.)
// It calls sh2_service to keep data flowing between host and sensor hub.
void BNO085_service(void)
{
    if (resetOccurred) {
        // Restart the flow of sensor reports
        resetOccurred = false;
        startReports();
    }
    
    // Service the sensor hub.
    // Sensor reports and event processing handled by callbacks.
    sh2_service();
}

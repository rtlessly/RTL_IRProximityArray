/*******************************************************************************
 * Header file for 2-sensor IR Range detector
 *
 * created 09 Jul 2016
 * by R. Terry Lessly
 *
 *******************************************************************************/
#ifndef _IRRangeDetector_H_
#define _IRRangeDetector_H_

#include <Arduino.h>
#include <Debug.h>
#include <EventSource.h>

// Maximum number of IR proximity sensors the component can handle
#define MAX_IR_SENSORS 6


//******************************************************************************
/// A component that represents an array of IR proximity sensors.
///
/// The classic application for IRProximityArray is an obstacle detector for a
/// robot. The IR proximity sensors are usually arranged in an arc or semi-
/// circular pattern facing in a specific direction. Up to 6 IR proximity sensors
/// can be used (more can be added by changing the MAX_IR_SENSORS value in
/// this header file). Sensors can be added via the constructor or by calling
/// the AddSensor() method. Sensors must be added in left-to-right order
/// (otherwise the readings won't be meaningful).
///
/// IRProximityArray monitors each sensor in the array and calculates a value
/// that represents the approximate direction the obstacle was detected. The
/// detection value is a signed 16-bit integer with -32767 indicating far-left
/// and +32767 indicating far-right. A value of zero indicates a straight-ahead
/// detection, while a value of -32678 means nothing was detected (represented by
/// the constant NO_DETECTION). Intermediate values indicate the degree to the
/// left (negative) or right (positive) an obstacle was detected. If you know
/// the physical geometry of the sensor array, you can convert the reading
/// value to a corresponding angle.
///
/// You can get a reading any time by calling the Read() method. In addition, an
/// event listener can be attached to monitor detection events. This component
/// fires the PROXIMITY_EVENT event whenever a change in the detection state
/// occurs. 
//******************************************************************************
class IRProximityArray : public EventSource
{
    //**************************************************************************
    // Constants
    //**************************************************************************
    public: static const float NO_DETECTION;

    //**************************************************************************
    // Class variables
    //**************************************************************************
    /// The Range detection event ID.
    public: static EVENT_ID PROXIMITY_EVENT;

    //**************************************************************************
    // Constructors
    //**************************************************************************
    public: IRProximityArray(int ir1Pin=0, int ir2Pin=0, int ir3Pin=0, int ir4Pin=0, int ir5Pin=0, int ir6Pin=0);

    //**************************************************************************
    // Public methods
    //**************************************************************************
    /// Adds an IR proximity sensor connected on the specified pin.
    /// IR sensors must be added in left-to-right order for the detector
    /// results to make sense.
    /// Returns the index in the sensor array where the sensor was added
    /// (0 to MAX_IR_SENSORS). A return value of -1 indicates the sensor could
    /// not be added (i.e., the array is full).
    public: int AddSensor(int pin);

    /// Takes a reading. If a change in reading occurs this method fires the PROXIMITY_EVENT event.
    public: float Read();

    /// Resets the array so that it can make a fresh detection
    public: void Reset();

    /// Polling method for the EventDispatcher to call
    public: virtual void Poll();

    //**************************************************************************
    // Internal state
    //**************************************************************************
    /// The array of IR sensors, ordered left-to-right.
    /// It is only necessary to store the pin number to which the sensor is connected.
    private: uint8_t _sensors[MAX_IR_SENSORS];

    /// Actual number of IR proximity sensors for this instance
    private: uint8_t _sensorCount;

    /// The last reading value taken
    private: float  _lastReading;

    // The reading 'distance' between adjacent sensors
    private: float _delta;
};

#endif


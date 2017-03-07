#define DEBUG 0

/*******************************************************************************
 * Implementation file for multi-sensor IR Proximity Array
 *
 * created 09 Jul 2016
 * by R. Terry Lessly
 *
 *******************************************************************************/
#include "IRProximityArray.h"

#define SENSITIVITY 20


static DebugHelper Debug("IRProximityArray");


const float IRProximityArray::NO_DETECTION = -99;

EVENT_ID IRProximityArray::PROXIMITY_EVENT = EventSource::GenerateEventID();


IRProximityArray::IRProximityArray(int ir1Pin, int ir2Pin, int ir3Pin, int ir4Pin, int ir5Pin, int ir6Pin)
{
    _sensorCount = 0;
    _lastReading = NO_DETECTION;

    for (int i = 0; i < MAX_IR_SENSORS; i++) _sensors[i] = 0;

    if (ir1Pin > 0) AddSensor(ir1Pin);
    if (ir2Pin > 0) AddSensor(ir2Pin);
    if (ir3Pin > 0) AddSensor(ir3Pin);
    if (ir4Pin > 0) AddSensor(ir4Pin);
    if (ir5Pin > 0) AddSensor(ir5Pin);
    if (ir6Pin > 0) AddSensor(ir6Pin);
}


/// Adds an IR sensor connected on the specified pin.
/// IR sensors must be added in left-to-right order for the results to make sense.
int IRProximityArray::AddSensor(int pin)
{
    int index = -1;

    if (_sensorCount < MAX_IR_SENSORS)
    {
        index = _sensorCount++;
        _sensors[index] = pin;
        pinMode(pin, INPUT);
        
        Debug.Log("AddSensor => IR sensor[%i] added on pin=%i", index, pin);
    }

    // Compute delta value of sensor array.
    // The delta value is essentially the resolution of sensor array, normalized 
    // over a range of 0 to 2. If the array has 5 sensors, then there a 4 'gaps' 
    // between the sensors. These gaps represent the reading 'distance' between 
    // adjacent sensors. The range of 0 to 2 is chosen because eventually the 
    // reading will be normalized between -1 and +1, which is a span of 2. 
    _delta =  2.0 / max(_sensorCount - 1, 1); // Use max() to avoid division by zero

    return index;
}


/// Resets the array so that a fresh detection can be made.
void IRProximityArray::Reset()
{
    _lastReading = NO_DETECTION;
}


float IRProximityArray::Read()
{
    // Accumulates the weighted value of the reading. Negative values are to the left
    // while positive values are to the right.
    float reading = 0;
    int triggeredCount = 0; // Count of sensors that triggered

    // Loop through all sensors to see which ones were triggered, accumulating
    // the reading along the way. The sensors are are weighted so that the left-most
    // sensor (at index 0) contributes a value of -1 to the reading, while the 
    // right-most sensor (at index _sensorCount-1) contributes a value of +1. 
    // Detectors in-between contribute corresponding weighted values between -1 
    // and +1. Because the delta value is normalized over a range of 0 and 2, if 
    // we multiply the sensor index by the delta value and then subtract 1 from 
    // that value we will get a weighted value for that sensor between -1 and +1 
    // as the sensor index progresses from left to right (i.e., 0 to _sensorCount-1).
    for (int i = 0; i < _sensorCount; i++)
    {
        if (_sensors[i] == 0) continue; // Just in case an undefined sensor slipped through

        // If the sensor is triggered
        if (digitalRead(_sensors[i]) == LOW)
        {
            triggeredCount++;

            // Add the sensor's weighted value to the accumulated value. 
            reading += (i * _delta) - 1;
        }
    }

    // If something was detected then normalize the reading to +/- 1
    // (negative values are to the left, positive to the right)
    // If nothing was detected, then set the reading to NO_DETECTION (-99)
    reading = (triggeredCount > 0) ? (reading / triggeredCount) : NO_DETECTION;

    return reading;
}


void IRProximityArray::Poll()
{
    float reading = Read();

    if (reading != _lastReading)
    {
        Debug.Log("Poll => reading=%f, _lastReading=%f", reading, _lastReading);

        _lastReading = reading;
        
        Event event(PROXIMITY_EVENT, reading);
        
        DispatchEvent(&event);
    }
}

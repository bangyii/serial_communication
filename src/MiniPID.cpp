/**
 * Code forked from author below. Changes made to take into account iteration time
 * 
 * Small, easy to use PID implementation with advanced controller capability.<br> 
 * Minimal usage:<br>
 * setPID(p,i,d); <br>
 * ...looping code...{ <br>
 * output=getOutput(sensorvalue,target); <br>
 * }
 * 
 * @see http://brettbeauregard.com/blog/2011/04/improving-the-beginners-pid-direction/improving-the-beginners-pid-introduction
 **/

#include "serial_communication/MiniPID.h"
#include <iostream>
//**********************************
//Constructor functions
//**********************************
MiniPID::MiniPID()
{
	init();
}

MiniPID::MiniPID(double p, double i, double d)
{
	init();
	P = p;
	I = i;
	D = d;
}
MiniPID::MiniPID(double p, double i, double d, double f)
{
	init();
	P = p;
	I = i;
	D = d;
	F = f;
}
void MiniPID::init()
{
	P = 0;
	I = 0;
	D = 0;
	F = 0;

	maxIOutput = 0;
	maxError = 0;
	errorSum = 0;
	maxOutput = 0;
	minOutput = 0;
	setpoint = 0;
	lastActual = 0;
	firstRun = true;
	reversed = false;
	outputRampRate = 0;
	outputDescentRate = 0;
	lastOutput = 0;
	outputFilter = 0;
	setpointRange = 0;
	deadTime = 0;
	frequency = 5.0;
}

//**********************************
//Configuration functions
//**********************************
/**
 * Configure the Proportional gain parameter. <br>
 * this responds quickly to changes in setpoint, and provides most of the initial driving force
 * to make corrections. <br>
 * Some systems can be used with only a P gain, and many can be operated with only PI.<br>
 * For position based controllers, this is the first parameter to tune, with I second. <br>
 * For rate controlled systems, this is often the second after F.
 *
 * @param p Proportional gain. Affects output according to <b>output+=P*(setpoint-current_value)</b>
 */
void MiniPID::setP(double p)
{
	P = p;
	checkSigns();
}

void MiniPID::setDeadtime(double val)
{
	deadTime = val;
}

void MiniPID::setFreq(double val)
{
	frequency = val;
}

/**
 * Changes the I parameter <br>
 * this is used for overcoming disturbances, and ensuring that the controller always gets to the control mode. 
 * Typically tuned second for "Position" based modes, and third for "Rate" or continuous based modes. <br>
 * Affects output through <b>output+=previous_errors*Igain ;previous_errors+=current_error</b>
 * 
 * @see {@link #setMaxIOutput(double) setMaxIOutput} for how to restrict
 *
 * @param i New gain value for the Integral term
 */
void MiniPID::setI(double i)
{
	if (I != 0)
	{
		errorSum = errorSum * I / i;
	}
	if (maxIOutput != 0)
	{
		maxError = maxIOutput / i;
	}
	I = i;
	checkSigns();
	/* Implementation note: 
	 * this scales the accumulated error to avoid output errors. 
	 * As an example doubling the I term cuts the accumulated error in half, which results in the 
	 * output change due to the I term constant during the transition. 
	 *
	 */
}

void MiniPID::setD(double d)
{
	D = d;
	checkSigns();
}

/**Configure the FeedForward parameter. <br>
 * this is excellent for Velocity, rate, and other	continuous control modes where you can 
 * expect a rough output value based solely on the setpoint.<br>
 * Should not be used in "position" based control modes.
 * 
 * @param f Feed forward gain. Affects output according to <b>output+=F*Setpoint</b>;
 */
void MiniPID::setF(double f)
{
	F = f;
	checkSigns();
}

/** Create a new PID object. 
 * @param p Proportional gain. Large if large difference between setpoint and target. 
 * @param i Integral gain.	Becomes large if setpoint cannot reach target quickly. 
 * @param d Derivative gain. Responds quickly to large changes in error. Small values prevents P and I terms from causing overshoot.
 */
void MiniPID::setPID(double p, double i, double d)
{
	P = p;
	I = i;
	D = d;
	checkSigns();
}

void MiniPID::setPID(double p, double i, double d, double f)
{
	P = p;
	I = i;
	D = d;
	F = f;
	checkSigns();
}

/**Set the maximum output value contributed by the I component of the system
 * this can be used to prevent large windup issues and make tuning simpler
 * @param maximum. Units are the same as the expected output value
 */
void MiniPID::setMaxIOutput(double maximum)
{
	/* Internally maxError and Izone are similar, but scaled for different purposes. 
	 * The maxError is generated for simplifying math, since calculations against 
	 * the max error are far more common than changing the I term or Izone. 
	 */
	maxIOutput = maximum;
	if (I != 0)
	{
		maxError = maxIOutput / I;
	}
}

/**Specify a maximum output. If a single parameter is specified, the minimum is 
 * set to (-maximum).
 * @param output 
 */
void MiniPID::setOutputLimits(double output) { setOutputLimits(-output, output); }

/**
 * Specify a maximum output.
 * @param minimum possible output value
 * @param maximum possible output value
 */
void MiniPID::setOutputLimits(double minimum, double maximum)
{
	if (maximum < minimum)
		return;
	maxOutput = maximum;
	minOutput = minimum;

	// Ensure the bounds of the I term are within the bounds of the allowable output swing
	if (maxIOutput == 0 || maxIOutput > (maximum - minimum))
	{
		setMaxIOutput(maximum - minimum);
	}
}

/** Set the operating direction of the PID controller
 * @param reversed Set true to reverse PID output
 */
void MiniPID::setDirection(bool reversed)
{
	this->reversed = reversed;
}

//**********************************
//Primary operating functions
//**********************************

/**Set the target for the PID calculations
 * @param setpoint
 */
void MiniPID::setSetpoint(double setpoint)
{
	this->setpoint = setpoint;
}

/** Used to skip this PID cycle. Required to prevent integral windup if many cycles are skipped
 * @return calculated output from the previous PID cycle
 **/
double MiniPID::skipCycle()
{
	prev_time = std::chrono::system_clock::now();
	return lastOutput;
}

/** Calculate the PID value needed to hit the target setpoint. 
* Automatically re-calculates the output at each call. 
* @param actual The monitored value
* @param target The target value
* @return calculated output value for driving the actual to the target 
*/
double MiniPID::getOutput(double actual, double setpoint)
{
	double output;
	double Poutput;
	double Ioutput;
	double Doutput;
	double Foutput;

	//Remember old errorSum for use in reverting errorSum if any limits are reached later on
	double oldErrorSum = errorSum;

	this->setpoint = setpoint;

	//Do the simple parts of the calculations
	double error = setpoint - actual;

	//If this is our first time running this  we don't actually _have_ a previous input or output.
	//For sensor, sanely assume it was exactly where it is now.
	//For last output, we can assume it's the current time-independent outputs.
	if (firstRun)
	{
		lastActual = actual;
		prevError = error;
		lastOutput = Poutput + Foutput;
		prev_time = std::chrono::system_clock::now();
		firstRun = false;
	}

	//Only run cycle when time passed is greater than 1/Hz
	if ((std::chrono::system_clock::now() - prev_time).count() / 1000000000.0 < 1 / frequency)
		return lastOutput;

	//Ramp the setpoint used for calculations if user has opted to do so
	if (setpointRange != 0)
	{
		setpoint = clamp(setpoint, actual - setpointRange, actual + setpointRange);
	}

	//Calculate F output. Notice, this depends only on the setpoint, and not the error.
	Foutput = F * setpoint;

	//Calculate P term
	Poutput = P * error;

	//Get time difference since last run
	float dt = (std::chrono::system_clock::now() - prev_time).count() / 1000000000.0;

	//Calculate D Term
	//If rate of change of error is positive, then the derivative term should be positive to track
	//target setpoint, as system is lagging behind
	float error_rate = (error - prevError) / dt;
	Doutput = D * error_rate;

	//The Iterm is more complex. There's several things to factor in to make it easier to deal with.
	// 1. maxIoutput restricts the amount of output contributed by the Iterm.
	// 2. prevent windup by not increasing errorSum if we're already running against our max Ioutput
	// 3. prevent windup by not increasing errorSum if output is output=maxOutput
	errorSum += error;
	Ioutput = I * errorSum * dt;

	//Case 2: Clamp IOutput to max allowed integral output
	if (maxIOutput != 0 && !bounded(Ioutput, -maxIOutput, maxIOutput))
	{
		Ioutput = clamp(Ioutput, -maxIOutput, maxIOutput);

		//Max Ioutput reached, clamp errorSum
		errorSum = oldErrorSum;
	}

	//And, finally, we can just add the terms up
	output = Foutput + Poutput + Ioutput + Doutput;

	//Restrict output to our specified output and ramp limits
	//Output decent rate should be negative
	if (outputRampRate != 0 && outputDescentRate != 0 && !bounded(output, lastOutput + outputDescentRate * dt, lastOutput + outputRampRate * dt))
	{
		//If output is positive, allow outputRampRate increase and outputDescentRate decrease
		if(output >= 0)
		{
			if(!bounded(output, lastOutput + outputDescentRate * dt, lastOutput + outputRampRate * dt))
			{ 
				output = clamp(output, lastOutput + outputDescentRate * dt, lastOutput + outputRampRate * dt);
				errorSum = oldErrorSum; 
			}
		}

		else
		{
			if(!bounded(output, lastOutput - outputRampRate * dt, lastOutput - outputDescentRate * dt))
			{
				output = clamp(output, lastOutput - outputRampRate * dt, lastOutput - outputDescentRate * dt);
				errorSum = oldErrorSum;
			}
		}
		
		// output = clamp(output, lastOutput + outputDescentRate * dt, lastOutput + outputRampRate * dt);

		//Prevent errorsum from increasing if ramp rate is already capped
		// errorSum = oldErrorSum;
	}

	if (minOutput != maxOutput && !bounded(output, minOutput, maxOutput))
	{
		output = clamp(output, minOutput, maxOutput);

		//Prevent errorsum from increasing if max output is already capped
		errorSum = oldErrorSum;
	}

	if (outputFilter != 0)
	{
		output = lastOutput * outputFilter + output * (1 - outputFilter);
	}

	lastOutput = output;
	prev_time = std::chrono::system_clock::now();
	prevError = error;
	return output;
}

/**
 * Calculates the PID value using the last provided setpoint and actual values
 * @return calculated output value for driving the actual to the target 
 */
double MiniPID::getOutput()
{
	return getOutput(lastActual, setpoint);
}

/**
 * 
 * @param actual
 * @return calculated output value for driving the actual to the target 
 */
double MiniPID::getOutput(double actual)
{
	return getOutput(actual, setpoint);
}

/**
 * Resets the controller. this erases the I term buildup, and removes D gain on the next loop.
 */
void MiniPID::reset()
{
	firstRun = true;
	errorSum = 0;
}

/**Set the maximum rate the output can increase per cycle. 
 * @param rate
 */
void MiniPID::setOutputRampRate(double rate)
{
	outputRampRate = rate;
}

/**Set the maximum rate the output can decrease per cycle.
 * @param rate
 **/
void MiniPID::setOutputDescentRate(double rate)
{
	outputDescentRate = rate;
}

/** Set a limit on how far the setpoint can be from the current position
 * <br>Can simplify tuning by helping tuning over a small range applies to a much larger range. 
 * <br>this limits the reactivity of P term, and restricts impact of large D term
 * during large setpoint adjustments. Increases lag and I term if range is too small.
 * @param range
 */
void MiniPID::setSetpointRange(double range)
{
	setpointRange = range;
}

/**Set a filter on the output to reduce sharp oscillations. <br>
 * 0.1 is likely a sane starting value. Larger values P and D oscillations, but force larger I values.
 * Uses an exponential rolling sum filter, according to a simple <br>
 * <pre>output*(1-strength)*sum(0..n){output*strength^n}</pre>
 * @param output valid between [0..1), meaning [current output only.. historical output only)
 */
void MiniPID::setOutputFilter(double strength)
{
	if (strength == 0 || bounded(strength, 0, 1))
	{
		outputFilter = strength;
	}
}

//**************************************
// Helper functions
//**************************************

/**
 * Forces a value into a specific range
 * @param value input value
 * @param min maximum returned value
 * @param max minimum value in range
 * @return Value if it's within provided range, min or max otherwise 
 */
double MiniPID::clamp(double value, double min, double max)
{
	if (value > max)
	{
		return max;
	}
	if (value < min)
	{
		return min;
	}
	return value;
}

/**
 * Test if the value is within the min and max, inclusive
 * @param value to test
 * @param min Minimum value of range
 * @param max Maximum value of range
 * @return
 */
bool MiniPID::bounded(double value, double min, double max)
{
	return (min < value) && (value < max);
}

/**
 * To operate correctly, all PID parameters require the same sign,
 * with that sign depending on the {@literal}reversed value
 */
void MiniPID::checkSigns()
{
	if (reversed)
	{ //all values should be below zero
		if (P > 0)
			P *= -1;
		if (I > 0)
			I *= -1;
		if (D > 0)
			D *= -1;
		if (F > 0)
			F *= -1;
	}
	else
	{ //all values should be above zero
		if (P < 0)
			P *= -1;
		if (I < 0)
			I *= -1;
		if (D < 0)
			D *= -1;
		if (F < 0)
			F *= -1;
	}
}

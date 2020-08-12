#include <serial_communication/adaptive_ctrl.h>
#include <math.h>

/**
 * Adaptive control algorithm implemented with reference to paper titled
 * "Adaptive Closed-loop Speed ontrol of BLDC Motorswith Applications to Multi-rotor Aerial Vehicles"
 * from URL: https://ieeexplore.ieee.org/document/7989610"
 **/

AdaptiveControl::AdaptiveControl(float _gainThresh, float _gainStep, float _biasThresh, float _biasStep, float _weight)
{
    gainUpdateThreshold = _gainThresh;
    biasUpdateThreshold = _biasThresh;
    gainUpdateStep = _gainStep;
    biasUpdateStep = _biasStep;
    weight = _weight;
}

AdaptiveControl::AdaptiveControl()
{
}

AdaptiveControl::~AdaptiveControl()
{
}

bool AdaptiveControl::setBias(float _bias)
{
    if (isBounded(_bias, 0, 1))
    {
        bias = _bias;
        return true;
    }

    return false;
}
bool AdaptiveControl::setGain(float _gain)
{
    if (isBounded(_gain, 0, 1))
    {

        gain = _gain;
        return true;
    }

    return false;
}
bool AdaptiveControl::setSignFilterWeight(float _weight)
{
    if (isBounded(_weight, 0, 1))
    {
        weight = _weight;
        return true;
    }

    return false;
}
float AdaptiveControl::getOutput(float current, float target)
{
    //Update filtered sign
    if (current > target)
        filtered_sgn = weight * filtered_sgn + (1 - weight);
    else
        filtered_sgn = weight * filtered_sgn - (1 - weight);

    //Update bias
    if (filtered_sgn > biasUpdateThreshold)
        bias += biasUpdateStep;
    else if (filtered_sgn < -biasUpdateThreshold)
        bias -= biasUpdateStep;
    clamp(bias, 0.0, 1.0);

    //Update gain
    if (fabs(filtered_sgn) > gainUpdateThreshold)
        gain += gainUpdateStep;
    else
        gain -= gainUpdateStep;
    clamp(gain, 0.0, 1.0);

    //Control output
    if (current > target)
        output = bias + gain;
    else if (current < target)
        output = bias - gain;
    else
        output = bias;
    clamp(output, 0.0, 1.0);

    //Scale output to min/max in the same direction
    if (target > 0)
        return output * maxOutput;
    else if (target < 0)
        return output * minOutput;

    return 0;
}
bool AdaptiveControl::setOutputLimits(float min, float max)
{
    if (min < max)
    {
        minOutput = min;
        maxOutput = max;
        return true;
    }

    return false;
}

bool AdaptiveControl::isBounded(float value, float min, float max)
{
    return (value >= min && value <= max);
}

void AdaptiveControl::clamp(float &value, float min, float max)
{
    if (value < min)
        value = min;
    if (value > max)
        value = max;
}

int AdaptiveControl::getSign(double val)
{
    return int(val / fabs(val));
}

bool AdaptiveControl::setGainUpdateThreshold(float _val)
{
    gainUpdateThreshold = _val;
}
bool AdaptiveControl::setBiasUpdateThreshold(float _val)
{
    biasUpdateThreshold = _val;
}
void AdaptiveControl::setBiasUpdateStep(float _val)
{
    biasUpdateStep = _val;
}
void AdaptiveControl::setGainUpdateStep(float _val)
{
    gainUpdateStep = _val;
}
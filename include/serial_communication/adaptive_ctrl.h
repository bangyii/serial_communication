#ifndef ADAPTIVE_CTRL_H
#define ADAPTIVE_CTRL_H

class AdaptiveControl
{
public:
    AdaptiveControl();
    AdaptiveControl(float _gainThresh, float _gainStep, float _biasThresh, float _biasStep, float _weight);
    ~AdaptiveControl();

    bool setBias(float _bias);
    bool setGain(float _gain);
    bool setSignFilterWeight(float _weight);
    bool setGainUpdateThreshold(float _val);
    bool setBiasUpdateThreshold(float _val);
    void setBiasUpdateStep(float _val);
    void setGainUpdateStep(float _val);
    float getOutput(float current, float target);
    bool setOutputLimits(float min, float max);


private:
    float output = 0;
    float bias = 0;
    float weight = 1.0/4.0;
    float gain = 0;
    float filtered_sgn = 0;
    float minOutput = 0;
    float maxOutput = 0;
    float biasUpdateThreshold = 0.75;
    float gainUpdateThreshold = 0.50;
    float biasUpdateStep = 1.0/1024.0;
    float gainUpdateStep = 1.0/1024.0;

    bool isBounded(float value, float min, float max);
    void clamp(float &value, float min, float max);
    int getSign(double val);
};


#endif
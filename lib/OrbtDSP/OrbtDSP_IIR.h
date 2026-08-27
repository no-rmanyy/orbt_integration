#pragma once

#include <Arduino.h>
#include <type_traits>

#include "OrbtDSP_Filter.h"

/**
 * OrbtDSP_IIR_1LP is a First-order low-pass IIR filter.
 *
 * OrbtDSP_IIR_1LP implements a simple first-order (one-pole) Infinite Impulse Response (IIR) low-pass filter
 * for float/double types, providing configurable cutoff frequency and filter update.
 *
 * OrbtDSP_IIR_2LP and OrbtDSP_IIR_4LP are Second-order and Fourth-order low-pass IIR filters.
 *
 * OrbtDSP_IIR_2LP implements a second-order biquad low-pass filter. OrbtDSP_IIR_4LP cascades two
 * second-order low-pass sections using Butterworth Q values for a fourth-order response.
 *
 * OrbtDSP_IIR_2BP is a Second-order band-pass IIR filter.
 *
 * OrbtDSP_IIR_2BP provides an implementation of a second-order Infinite Impulse Response (IIR) band-pass filter
 * for float/double types, allowing specification of bandwidth and center frequency for signal processing applications.
 *
 * OrbtDSP_IIR_2NF is a Second-order notch IIR filter.
 *
 * OrbtDSP_IIR_2NF provides an implementation of a second-order Infinite Impulse Response (IIR) notch filter
 * for float/double types, allowing specification of bandwidth and center frequency for removing narrowband noise.
 */



// ------------------------------------------------------------------------------------------------
// OrbtDSP_IIR_1LP
// ------------------------------------------------------------------------------------------------
template <typename T = float>
class OrbtDSP_IIR_1LP : public OrbtDSP_Filter<T> {
    static_assert(std::is_same<T, float>::value || std::is_same<T, double>::value,
                  "OrbtDSP_IIR_1LP only supports float and double");

public:
    OrbtDSP_IIR_1LP() = default;

    OrbtDSP_IIR_1LP(T sampleRate, T filterFrequency) {
        setLowPass(sampleRate, filterFrequency);
    }

    explicit OrbtDSP_IIR_1LP(T alpha) {
        setAlpha(alpha);
    }

    ~OrbtDSP_IIR_1LP() = default;

    bool setLowPass(T sampleRate, T filterFrequency) {
        if (sampleRate <= T{} || filterFrequency <= T{}) {
            setAlpha(T{1});
            return false;
        }

        if (filterFrequency >= (sampleRate / T{2})) {
            setAlpha(T{1});
            return true;
        }

        const T pi = T{3.14159265358979323846};
        const T alpha = T{1} - exp((-T{2} * pi * filterFrequency) / sampleRate);
        setAlpha(alpha);
        return true;
    }

    void setAlpha(T alpha) {
        if (alpha < T{}) {
            _alpha = T{};
        } else if (alpha > T{1}) {
            _alpha = T{1};
        } else {
            _alpha = alpha;
        }
    }

    void reset(T value = T{}) override {
        _acc = value;
        _initialized = false;
    }

    T process(T input) override {
        if (!_initialized) {
            _acc = input;
            _initialized = true;
            return _acc;
        }

        _acc = (input * _alpha) + (_acc * (T{1} - _alpha));
        return _acc;
    }

    T output() const override {
        return _acc;
    }

    T alpha() const {
        return _alpha;
    }

private:
    T _alpha = T{1};
    T _acc = T{};
    bool _initialized = false;
};

using OrbtDSP_IIR_1LPf = OrbtDSP_IIR_1LP<float>;
using OrbtDSP_IIR_1LPd = OrbtDSP_IIR_1LP<double>;

// ------------------------------------------------------------------------------------------------
// OrbtDSP_IIR_2LP
// ------------------------------------------------------------------------------------------------
template <typename T = float>
class OrbtDSP_IIR_2LP : public OrbtDSP_Filter<T> {
    static_assert(std::is_same<T, float>::value || std::is_same<T, double>::value,
                  "OrbtDSP_IIR_2LP only supports float and double");

public:
    OrbtDSP_IIR_2LP() = default;

    OrbtDSP_IIR_2LP(T sampleRate, T filterFrequency) {
        setLowPass(sampleRate, filterFrequency);
    }

    OrbtDSP_IIR_2LP(T sampleRate, T filterFrequency, T q) {
        setLowPass(sampleRate, filterFrequency, q);
    }

    OrbtDSP_IIR_2LP(T b0, T b1, T b2, T a1, T a2) {
        setCoefficients(b0, b1, b2, a1, a2);
    }

    ~OrbtDSP_IIR_2LP() = default;

    void setCoefficients(T b0, T b1, T b2, T a1, T a2) {
        _b0 = b0;
        _b1 = b1;
        _b2 = b2;
        _a1 = a1;
        _a2 = a2;
    }

    bool setLowPass(T sampleRate, T filterFrequency) {
        return setLowPass(sampleRate, filterFrequency, T{0.70710678118654752440});
    }

    bool setLowPass(T sampleRate, T filterFrequency, T q) {
        if (sampleRate <= T{} || filterFrequency <= T{} || q <= T{}) {
            setCoefficients(T{1}, T{}, T{}, T{}, T{});
            return false;
        }

        if (filterFrequency >= (sampleRate / T{2})) {
            setCoefficients(T{1}, T{}, T{}, T{}, T{});
            return true;
        }

        const T pi = T{3.14159265358979323846};
        const T omega = (T{2} * pi * filterFrequency) / sampleRate;
        const T cosOmega = cos(omega);
        const T alpha = sin(omega) / (T{2} * q);
        const T a0 = T{1} + alpha;
        const T oneMinusCos = T{1} - cosOmega;

        setCoefficients((oneMinusCos / T{2}) / a0,
                        oneMinusCos / a0,
                        (oneMinusCos / T{2}) / a0,
                        (-T{2} * cosOmega) / a0,
                        (T{1} - alpha) / a0);
        return true;
    }

    void reset(T value = T{}) override {
        _x1 = value;
        _x2 = value;
        _y1 = value;
        _y2 = value;
    }

    T process(T input) override {
        const T output = (_b0 * input) + (_b1 * _x1) + (_b2 * _x2) - (_a1 * _y1) - (_a2 * _y2);

        _x2 = _x1;
        _x1 = input;
        _y2 = _y1;
        _y1 = output;

        return output;
    }

    T output() const override {
        return _y1;
    }

private:
    T _b0 = T{1};
    T _b1 = T{};
    T _b2 = T{};
    T _a1 = T{};
    T _a2 = T{};

    T _x1 = T{};
    T _x2 = T{};
    T _y1 = T{};
    T _y2 = T{};
};

using OrbtDSP_IIR_2LPf = OrbtDSP_IIR_2LP<float>;
using OrbtDSP_IIR_2LPd = OrbtDSP_IIR_2LP<double>;

// ------------------------------------------------------------------------------------------------
// OrbtDSP_IIR_4LP
// ------------------------------------------------------------------------------------------------
template <typename T = float>
class OrbtDSP_IIR_4LP : public OrbtDSP_Filter<T> {
    static_assert(std::is_same<T, float>::value || std::is_same<T, double>::value,
                  "OrbtDSP_IIR_4LP only supports float and double");

public:
    OrbtDSP_IIR_4LP() = default;

    OrbtDSP_IIR_4LP(T sampleRate, T filterFrequency) {
        setLowPass(sampleRate, filterFrequency);
    }

    ~OrbtDSP_IIR_4LP() = default;

    bool setLowPass(T sampleRate, T filterFrequency) {
        const bool section1Ok = _section1.setLowPass(sampleRate, filterFrequency, T{0.54119610014619701222});
        const bool section2Ok = _section2.setLowPass(sampleRate, filterFrequency, T{1.30656296487637652774});
        return section1Ok && section2Ok;
    }

    void reset(T value = T{}) override {
        _section1.reset(value);
        _section2.reset(value);
    }

    T process(T input) override {
        return _section2.process(_section1.process(input));
    }

    T output() const override {
        return _section2.output();
    }

private:
    OrbtDSP_IIR_2LP<T> _section1;
    OrbtDSP_IIR_2LP<T> _section2;
};

using OrbtDSP_IIR_4LPf = OrbtDSP_IIR_4LP<float>;
using OrbtDSP_IIR_4LPd = OrbtDSP_IIR_4LP<double>;


// ------------------------------------------------------------------------------------------------
// OrbtDSP_IIR_2BP
// ------------------------------------------------------------------------------------------------
template <typename T = float>
class OrbtDSP_IIR_2BP : public OrbtDSP_Filter<T> {
    static_assert(std::is_same<T, float>::value || std::is_same<T, double>::value,
                  "OrbtDSP_IIR_2BP only supports float and double");

public:
    OrbtDSP_IIR_2BP() = default;

    OrbtDSP_IIR_2BP(T sampleRate, T filterFrequency, T bandwidth) {
        setBandPass(sampleRate, filterFrequency, bandwidth);
    }

    OrbtDSP_IIR_2BP(T b0, T b1, T b2, T a1, T a2) {
        setCoefficients(b0, b1, b2, a1, a2);
    }

    ~OrbtDSP_IIR_2BP() = default;

    void setCoefficients(T b0, T b1, T b2, T a1, T a2) {
        _b0 = b0;
        _b1 = b1;
        _b2 = b2;
        _a1 = a1;
        _a2 = a2;
    }

    bool setBandPass(T sampleRate, T filterFrequency, T bandwidth) {
        if (sampleRate <= T{} || filterFrequency <= T{} || bandwidth <= T{} ||
            filterFrequency >= (sampleRate / T{2})) {
            setCoefficients(T{1}, T{}, T{}, T{}, T{});
            return false;
        }

        const T pi = T{3.14159265358979323846};
        const T omega = (T{2} * pi * filterFrequency) / sampleRate;
        const T q = filterFrequency / bandwidth;
        const T alpha = sin(omega) / (T{2} * q);
        const T a0 = T{1} + alpha;

        setCoefficients(alpha / a0,
                        T{} / a0,
                        -alpha / a0,
                        (-T{2} * cos(omega)) / a0,
                        (T{1} - alpha) / a0);
        return true;
    }

    void reset(T value = T{}) override {
        _x1 = value;
        _x2 = value;
        _y1 = value;
        _y2 = value;
    }

    T process(T input) override {
        const T output = (_b0 * input) + (_b1 * _x1) + (_b2 * _x2) - (_a1 * _y1) - (_a2 * _y2);

        _x2 = _x1;
        _x1 = input;
        _y2 = _y1;
        _y1 = output;

        return output;
    }

    T output() const override {
        return _y1;
    }

private:
    T _b0 = T{1};
    T _b1 = T{};
    T _b2 = T{};
    T _a1 = T{};
    T _a2 = T{};

    T _x1 = T{};
    T _x2 = T{};
    T _y1 = T{};
    T _y2 = T{};
};

using OrbtDSP_IIR_2BPf = OrbtDSP_IIR_2BP<float>;
using OrbtDSP_IIR_2BPd = OrbtDSP_IIR_2BP<double>;


// ------------------------------------------------------------------------------------------------
// OrbtDSP_IIR_2NF
// ------------------------------------------------------------------------------------------------
template <typename T = float>
class OrbtDSP_IIR_2NF : public OrbtDSP_Filter<T> {
    static_assert(std::is_same<T, float>::value || std::is_same<T, double>::value,
                  "OrbtDSP_IIR_2NF only supports float and double");

public:
    OrbtDSP_IIR_2NF() = default;

    OrbtDSP_IIR_2NF(T sampleRate, T filterFrequency, T bandwidth) {
        setNotch(sampleRate, filterFrequency, bandwidth);
    }

    OrbtDSP_IIR_2NF(T b0, T b1, T b2, T a1, T a2) {
        setCoefficients(b0, b1, b2, a1, a2);
    }

    ~OrbtDSP_IIR_2NF() = default;

    void setCoefficients(T b0, T b1, T b2, T a1, T a2) {
        _b0 = b0;
        _b1 = b1;
        _b2 = b2;
        _a1 = a1;
        _a2 = a2;
    }

    bool setNotch(T sampleRate, T filterFrequency, T bandwidth) {
        if (sampleRate <= T{} || filterFrequency <= T{} || bandwidth <= T{} ||
            filterFrequency >= (sampleRate / T{2})) {
            setCoefficients(T{1}, T{}, T{}, T{}, T{});
            return false;
        }

        const T pi = T{3.14159265358979323846};
        const T omega = (T{2} * pi * filterFrequency) / sampleRate;
        const T cosOmega = cos(omega);
        const T q = filterFrequency / bandwidth;
        const T alpha = sin(omega) / (T{2} * q);
        const T a0 = T{1} + alpha;

        setCoefficients(T{1} / a0,
                        (-T{2} * cosOmega) / a0,
                        T{1} / a0,
                        (-T{2} * cosOmega) / a0,
                        (T{1} - alpha) / a0);
        return true;
    }

    void reset(T value = T{}) override {
        _x1 = value;
        _x2 = value;
        _y1 = value;
        _y2 = value;
    }

    T process(T input) override {
        const T output = (_b0 * input) + (_b1 * _x1) + (_b2 * _x2) - (_a1 * _y1) - (_a2 * _y2);

        _x2 = _x1;
        _x1 = input;
        _y2 = _y1;
        _y1 = output;

        return output;
    }

    T output() const override {
        return _y1;
    }

private:
    T _b0 = T{1};
    T _b1 = T{};
    T _b2 = T{};
    T _a1 = T{};
    T _a2 = T{};

    T _x1 = T{};
    T _x2 = T{};
    T _y1 = T{};
    T _y2 = T{};
};

using OrbtDSP_IIR_2NFf = OrbtDSP_IIR_2NF<float>;
using OrbtDSP_IIR_2NFd = OrbtDSP_IIR_2NF<double>;


#pragma once

#include <Arduino.h>
#include <type_traits>

template <typename T = float>
class OrbtDSP_Filter {
    static_assert(std::is_same<T, float>::value || std::is_same<T, double>::value,
                  "OrbtDSP_Filter only supports float and double");

public:
    virtual ~OrbtDSP_Filter() = default;

    virtual void reset(T value = T{}) = 0;
    virtual T process(T input) = 0;
    virtual T output() const = 0;
};

template <typename T = float>
class OrbtDSP_Passthru : public OrbtDSP_Filter<T> {
    static_assert(std::is_same<T, float>::value || std::is_same<T, double>::value,
                  "OrbtDSP_Passthru only supports float and double");

public:
    OrbtDSP_Passthru() = default;

    ~OrbtDSP_Passthru() override = default;

    void reset(T value = T{}) override {
        _output = value;
    }

    T process(T input) override {
        _output = input;
        return _output;
    }

    T output() const override {
        return _output;
    }

private:
    T _output = T{};
};

using OrbtDSP_PassthruF = OrbtDSP_Passthru<float>;
using OrbtDSP_PassthruD = OrbtDSP_Passthru<double>;


#pragma once

#include <Arduino.h>
//#include <WiFi.h>
//#include <WiFiUdp.h>
//#include <map>
#include <string>

class Telemetry {
public:
    /**
     * @brief Virtual destructor to ensure proper cleanup of derived classes
     */
    virtual ~Telemetry() = default;

    //  /**
    //  * @brief Initialize the telemetry system
    //  *
    //  * This method should be called once during system initialization to set up
    //  * any required hardware, configurations, or initial states.
    //  */
    // virtual void begin() = 0;

    /**
     * @brief Check if the telemetry system is ready
     *
     * This method should be called to check if the telemetry system is ready to send data.
     *
     * @return true if the telemetry system is ready, false otherwise
     */
    virtual bool isReady() = 0;

    /**
     * @brief Begin a new frame
     *
     * This method should be called to begin a new frame.
     */
    virtual void beginFrame() = 0;

    /**
     * @brief Send the current frame
     *
     * This method should be called to send the current frame.
     */
    virtual void sendFrame() = 0;

    /*
     * @brief Output a metric
     *
     * This method should be called to output a metric.
     *
     * @param key the key of the metric
     * @param value the value of the metric
     */
    virtual void output_metric(const std::string& key, double value) = 0;
    virtual void output_metric(const std::string& key, int value) = 0;
    virtual void output_metric(const std::string& key, int64_t value) = 0;
    virtual void output_metric(const std::string& key, uint64_t value) = 0;

    /**
     * @brief Output a metric with a unit
     *
     * This method should be called to output a metric with a unit.
     *
     * @param key the key of the metric
     * @param value the value of the metric
     * @param unit the unit of the metric
     */
    virtual void output_metric(const std::string& key, double value, const std::string& unit) = 0;
    virtual void output_metric(const std::string& key, int value, const std::string& unit) = 0;
    virtual void output_metric(const std::string& key, int64_t value, const std::string& unit) = 0;
    virtual void output_metric(const std::string& key, uint64_t value, const std::string& unit) = 0;

    /**
     * @brief Log a message
     *
     * This method should be called to log a message.
     *
     * @param message the message to log
     */
    virtual void log(const std::string& message) = 0;

protected:
    /**
     * @brief Protected constructor to prevent direct instantiation
     *
     * Only derived classes can be instantiated.
     */
    Telemetry() = default;
};
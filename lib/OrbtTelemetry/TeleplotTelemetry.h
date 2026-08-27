#pragma once

#include "Telemetry.h"
#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <map>
#include <string>

class TeleplotTelemetry : public Telemetry {
protected:
    WiFiUDP _udpClient;
    const char* _serverAddress;
    uint16_t _serverPort;
    uint32_t _currentFrameTime_ms;
    bool _frameStarted;

    template<typename T>
    void _output_metric_impl(const std::string& key, T value);

public:
    TeleplotTelemetry(const char* address, uint16_t port);

    bool isReady() override;
    void beginFrame() override;
    void sendFrame() override;

    void output_metric(const std::string& key, double value) override;
    void output_metric(const std::string& key, int value) override;
    void output_metric(const std::string& key, int64_t value) override;
    void output_metric(const std::string& key, uint64_t value) override;

    void output_metric(const std::string& key, double value, const std::string& unit) override;
    void output_metric(const std::string& key, int value, const std::string& unit) override;
    void output_metric(const std::string& key, int64_t value, const std::string& unit) override;
    void output_metric(const std::string& key, uint64_t value, const std::string& unit) override;

    void log(const std::string& message) override;
};
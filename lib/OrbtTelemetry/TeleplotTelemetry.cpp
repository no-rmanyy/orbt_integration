#include "TeleplotTelemetry.h"

// Constructor
TeleplotTelemetry::TeleplotTelemetry(const char* address, uint16_t port)
    : _serverAddress(address), _serverPort(port) {
        _frameStarted = false;
}

// Public methods
void TeleplotTelemetry::sendFrame() {
    _udpClient.endPacket();
}

void TeleplotTelemetry::beginFrame() {
    _frameStarted = true;
    _currentFrameTime_ms = esp_timer_get_time() / 1000;
    _udpClient.beginPacket(_serverAddress, _serverPort);
}

bool TeleplotTelemetry::isReady() {
    _frameStarted = false;
    _udpClient.endPacket();
    return true;
}

void TeleplotTelemetry::log(const std::string& message) {
}

void TeleplotTelemetry::output_metric(const std::string& key, double value) {
    _output_metric_impl(key, value);
}

void TeleplotTelemetry::output_metric(const std::string& key, int value) {
    _output_metric_impl(key, value);
}

void TeleplotTelemetry::output_metric(const std::string& key, int64_t value) {
    _output_metric_impl(key, value);
}

void TeleplotTelemetry::output_metric(const std::string& key, uint64_t value) {
    _output_metric_impl(key, value);
}

// Teleplot doesn't support units, so we just pass the value through
void TeleplotTelemetry::output_metric(const std::string& key, double value, const std::string& unit) {
    _output_metric_impl(key, value);
}

void TeleplotTelemetry::output_metric(const std::string& key, int64_t value, const std::string& unit) {
    _output_metric_impl(key, value);
}

void TeleplotTelemetry::output_metric(const std::string& key, uint64_t value, const std::string& unit) {
    _output_metric_impl(key, value);
}

void TeleplotTelemetry::output_metric(const std::string& key, int value, const std::string& unit) {
    _output_metric_impl(key, value);
}

// Private methods
template<typename T>
void TeleplotTelemetry::_output_metric_impl(const std::string& key, T value)
{
    if(!_frameStarted) {
        log("ERROR: TeleplotTelemetry::output_metric can't be called out of frame\n");
        return;
    }

    _udpClient.print(key.c_str());
    _udpClient.print(":");
    _udpClient.print(_currentFrameTime_ms);
    _udpClient.print(":");
    _udpClient.println(value);
}
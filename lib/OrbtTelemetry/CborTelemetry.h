#pragma once

#include "Telemetry.h"
#include <Arduino.h>
#include <WiFi.h>
#include <AsyncUDP.h>
#include <array>
#include <map>
#include <string>
#include <vector>
#if defined(__has_include)
#  if __has_include(<tinycbor/cbor.h>)
#    include <tinycbor/cbor.h>
#  elif __has_include(<cbor.h>)
#    include <cbor.h>
#  else
#    error "TinyCBOR headers not found. Please add tinycbor to include path."
#  endif
#else
#  include <tinycbor/cbor.h>
#endif

//#define CBOR_TELEMETRY_PACK_ENABLE
//#define CBOR_TELEMETRY_TIMING_ENABLE

// No Pack
// CBOR timing over 100 frames: begin=10us max=67us, send=1054us max=2462us, connect=8us max=76us
// CBOR timing detail: keymap=0us max=0us (0 calls), encode=173us max=546us, transport=846us max=2255us
// CBOR timing io: pack=838us max=2247us, udp=828us max=2236us (100 calls), tcp=0us max=0us (0 calls), log=0us max=0us (0 calls)

// Pack
// CBOR timing over 100 frames: begin=7us max=74us, send=466us max=1785us, connect=8us max=148us
// CBOR timing detail: keymap=0us max=0us (0 calls), encode=189us max=1085us, transport=246us max=1558us
// CBOR timing io: pack=226us max=1547us, udp=925us max=1489us (20 calls), tcp=0us max=0us (0 calls), log=0us max=0us (0 calls)

class CborTelemetry : public Telemetry {
public:
	enum class ConnectionType {
		TCP,
		UDP
	};

protected:
	// Connection and transport
	const uint32_t _sendHeaderEveryNFrames = 1000;
	mutable WiFiClient _tcpClient;
	AsyncUDP _udpClient;
	ConnectionType _connType;
	const char* _serverAddress;
	uint16_t _serverPort;
	IPAddress _serverIP;
	bool _connected = false;
	uint32_t _lastReconnectAttempt = 0;
	const uint32_t _reconnectInterval = 5000; // 5 seconds
	const size_t _maxUdpPacketSize = 1400; // Maximum UDP packet size to avoid fragmentation

	// Metric registry and frame data
	struct MetricState {
		double floatValue = 0.0;
		int64_t intValue = 0;
		std::string unit;
		bool isFloat = false;
		bool present = false;
		bool activeThisFrame = false;
	};

	std::map<std::string, uint8_t> _keyToId;
	std::array<MetricState, 256> _metrics;
	std::vector<uint8_t> _activeIds;
	uint8_t _nextId = 0;
	uint32_t _sendCount = 0;

	// Reusable TX buffer to minimize heap churn
	std::vector<uint8_t> _txBuf;

	// Optional packet packing (compile-time)
	// Define CBOR_TELEMETRY_PACK_ENABLE to aggregate multiple CBOR messages into
	// an indefinite-length CBOR array and send only when approaching MTU or at
	// the end of the frame. Target size is _maxUdpPacketSize.
#ifdef CBOR_TELEMETRY_PACK_ENABLE
	static constexpr bool _packEnable = true;
#else
	static constexpr bool _packEnable = false;
#endif
	std::vector<uint8_t> _packBuf;  // holds CBOR indefinite array of messages when packing
	void _ensurePackInit();
	void _flushPackIfNeeded(bool endOfFrame);
	void _packOrSend(const uint8_t* data, size_t len);

	// No fallback encoder — TinyCBOR is required.

	bool _ensureConnected();
	void _disconnect();

	void _sendUDP(const uint8_t* data, size_t len);
	void _sendTCP(const uint8_t* data, size_t len);
	bool _validateUdpPacketSize(size_t len);

	void _sendKeyMappingCbor();
	void _buildAndSendFrameCbor();
	void _sendLogCbor(const std::string& message);
	bool _getOrCreateMetricId(const std::string& key, bool isFloat, uint8_t& id);
	MetricState& _markMetricActive(uint8_t id);

public:
	CborTelemetry(const char* address, uint16_t port, ConnectionType type = ConnectionType::TCP);

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

	void setConnectionType(ConnectionType type);
};



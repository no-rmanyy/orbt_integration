#include "CborTelemetry.h"

CborTelemetry::CborTelemetry(const char* address, uint16_t port, ConnectionType type)
	: _connType(type), _serverAddress(address), _serverPort(port) {
	// Improve WiFi throughput/reliability for frequent UDP sends
	WiFi.setSleep(false);
	// Pre-size a reasonable TX buffer to reduce reallocations
	_txBuf.resize(2048);
	_activeIds.reserve(64);
}

// Public methods
void CborTelemetry::sendFrame() {
	if (!_ensureConnected()) return;

	// Send key mapping every 100 sends (or when new keys are added, _sendCount reset)
	if (_sendCount++ % _sendHeaderEveryNFrames == 0) {
		_sendKeyMappingCbor();
	}
	_buildAndSendFrameCbor();
	// Do not flush at end-of-frame when packing is enabled; only flush on MTU overflow.
}

void CborTelemetry::beginFrame() {
	for (uint8_t id : _activeIds) {
		auto& metric = _metrics[id];
		metric.present = false;
		metric.activeThisFrame = false;
		metric.unit.clear();
	}
	_activeIds.clear();
}

bool CborTelemetry::isReady() {
	if (!_connected) {
		return false;
	}

	if (_connType == ConnectionType::TCP) {
		return _tcpClient.connected();
	}

	return true; // UDP is always "connected" once initialized
}

void CborTelemetry::log(const std::string& message) {
	if (!_ensureConnected()) return;
	_sendLogCbor(message);
}

bool CborTelemetry::_getOrCreateMetricId(const std::string& key, bool isFloat, uint8_t& id) {
	auto it = _keyToId.find(key);
	if (it == _keyToId.end()) {
		if (_keyToId.size() >= _metrics.size()) {
			return false;
		}

		id = _nextId++;
		_keyToId.emplace(key, id);
		_sendCount = 0;
	} else {
		id = it->second;
	}

	_metrics[id].isFloat = isFloat;
	return true;
}

CborTelemetry::MetricState& CborTelemetry::_markMetricActive(uint8_t id) {
	auto& metric = _metrics[id];
	if (!metric.activeThisFrame) {
		metric.activeThisFrame = true;
		_activeIds.push_back(id);
	}
	metric.present = true;
	return metric;
}

void CborTelemetry::output_metric(const std::string& key, double value) {
	uint8_t id;
	if (!_getOrCreateMetricId(key, true, id)) return;
	auto& metric = _markMetricActive(id);
	metric.floatValue = value;
	metric.isFloat = true;
}

void CborTelemetry::output_metric(const std::string& key, int64_t value) {
	uint8_t id;
	if (!_getOrCreateMetricId(key, false, id)) return;
	auto& metric = _markMetricActive(id);
	metric.intValue = value;
	metric.isFloat = false;
}

void CborTelemetry::output_metric(const std::string& key, uint64_t value) {
	output_metric(key, static_cast<int64_t>(value));
}

void CborTelemetry::output_metric(const std::string& key, int value) {
	output_metric(key, static_cast<int64_t>(value));
}

void CborTelemetry::output_metric(const std::string& key, double value, const std::string& unit) {
	uint8_t id;
	if (!_getOrCreateMetricId(key, true, id)) return;
	auto& metric = _markMetricActive(id);
	metric.floatValue = value;
	metric.isFloat = true;
	metric.unit = unit;
}

void CborTelemetry::output_metric(const std::string& key, int64_t value, const std::string& unit) {
	uint8_t id;
	if (!_getOrCreateMetricId(key, false, id)) return;
	auto& metric = _markMetricActive(id);
	metric.intValue = value;
	metric.isFloat = false;
	metric.unit = unit;
}

void CborTelemetry::output_metric(const std::string& key, uint64_t value, const std::string& unit) {
	output_metric(key, static_cast<int64_t>(value), unit);
}

void CborTelemetry::output_metric(const std::string& key, int value, const std::string& unit) {
	output_metric(key, static_cast<int64_t>(value), unit);
}

void CborTelemetry::setConnectionType(ConnectionType type) {
	if (type == _connType) {
		return;
	}
	_disconnect();
	_connType = type;
	_connected = false;
	_lastReconnectAttempt = 0;
	_ensureConnected();
}

// Private methods
bool CborTelemetry::_ensureConnected() {
	if (_connected) {
		if (_connType == ConnectionType::TCP && _tcpClient.connected()) {
			return true;
		}
		if (_connType == ConnectionType::UDP) {
			// Ensure we have a valid destination IP
			if (!_serverIP) {
				IPAddress ipParsed;
				if (ipParsed.fromString(_serverAddress)) {
					_serverIP = ipParsed;
				} else {
					IPAddress resolved;
					if (WiFi.hostByName(_serverAddress, resolved) == 1) {
						_serverIP = resolved;
					}
				}
			}
			return static_cast<bool>(_serverIP);
		}
	}

	uint32_t currentTime = millis();
	if (currentTime - _lastReconnectAttempt < _reconnectInterval) {
		return false;
	}
	_lastReconnectAttempt = currentTime;

	if (_connType == ConnectionType::TCP) {
		if (_tcpClient.connected()) {
			_tcpClient.stop();
		}
		if (_tcpClient.connect(_serverAddress, _serverPort)) {
			_connected = true;
			return true;
		}
	} else {  // UDP (connectionless, but resolve endpoint)
		IPAddress ipParsed;
		if (ipParsed.fromString(_serverAddress)) {
			_serverIP = ipParsed;
		} else {
			IPAddress resolved;
			if (WiFi.hostByName(_serverAddress, resolved) == 1) {
				_serverIP = resolved;
			}
		}
		_connected = static_cast<bool>(_serverIP);
		return _connected;
	}

	_connected = false;
	return false;
}

void CborTelemetry::_disconnect() {
	if (_connType == ConnectionType::TCP) {
		_tcpClient.stop();
	} else {
		_udpClient.close();
	}
	_connected = false;
}

bool CborTelemetry::_validateUdpPacketSize(size_t len) {
	return len < _maxUdpPacketSize;
}

void CborTelemetry::_sendUDP(const uint8_t* data, size_t len) {
	if (!_validateUdpPacketSize(len)) {
		// Truncate to fit
		len = _maxUdpPacketSize - 1;
	}
	if (!_serverIP) {
		// Best-effort resolve if needed
		IPAddress resolved;
		if (WiFi.hostByName(_serverAddress, resolved) == 1) {
			_serverIP = resolved;
		}
	}
	if (_serverIP) {
        //Serial.printf("UDP: %d\n", len);
		_udpClient.writeTo(data, len, _serverIP, _serverPort);
	}
}

void CborTelemetry::_sendTCP(const uint8_t* data, size_t len) {
	_tcpClient.write(data, len);
}

void CborTelemetry::_sendKeyMappingCbor() {
	// Estimate a reasonable buffer size
	size_t est = 64 + (_keyToId.size() * 48);
	if (_txBuf.size() < est) {
		_txBuf.resize(est);
	}

	CborEncoder enc;
	cbor_encoder_init(&enc, _txBuf.data(), _txBuf.size(), 0);

	CborEncoder topArr;
	CborEncoder entriesArr;
	cbor_encoder_create_array(&enc, &topArr, 2);
	cbor_encode_text_stringz(&topArr, "K");
	cbor_encoder_create_array(&topArr, &entriesArr, _keyToId.size());

	for (const auto& pair : _keyToId) {
		const std::string& key = pair.first;
		uint8_t id = pair.second;
		const auto& metric = _metrics[id];

		CborEncoder entryArr;
		cbor_encoder_create_array(&entriesArr, &entryArr, 4);
		cbor_encode_text_string(&entryArr, key.c_str(), key.size());
		cbor_encode_uint(&entryArr, id);
		cbor_encode_uint(&entryArr, metric.isFloat ? 1 : 0);
		if (!metric.unit.empty()) {
			const std::string& u = metric.unit;
			cbor_encode_text_string(&entryArr, u.c_str(), u.size());
		} else {
			cbor_encode_null(&entryArr);
		}
		cbor_encoder_close_container(&entriesArr, &entryArr);
	}

	cbor_encoder_close_container(&topArr, &entriesArr);
	cbor_encoder_close_container(&enc, &topArr);

	size_t used = cbor_encoder_get_buffer_size(&enc, _txBuf.data());
	if (used == 0) return;
	if (_connType == ConnectionType::UDP) {
		_packOrSend(_txBuf.data(), used);
	} else {
		_sendTCP(_txBuf.data(), used);
	}
}

void CborTelemetry::_buildAndSendFrameCbor() {
	size_t est = 64 + (_activeIds.size() * 16);
	if (_txBuf.size() < est) {
		_txBuf.resize(est);
	}

	CborEncoder enc;
	cbor_encoder_init(&enc, _txBuf.data(), _txBuf.size(), 0);

	CborEncoder topArr;
	CborEncoder valuesArr;
	cbor_encoder_create_array(&enc, &topArr, 3);
	cbor_encode_text_stringz(&topArr, "V");
	uint64_t ts_ms = static_cast<uint64_t>(esp_timer_get_time() / 1000);
	cbor_encode_uint(&topArr, ts_ms);

	cbor_encoder_create_array(&topArr, &valuesArr, _activeIds.size());

	for (uint8_t id : _activeIds) {
		const auto& metric = _metrics[id];
		if (!metric.present) {
			continue;
		}

		CborEncoder entryArr;
		cbor_encoder_create_array(&valuesArr, &entryArr, 2);
		cbor_encode_uint(&entryArr, id);
		if (metric.isFloat) {
			cbor_encode_double(&entryArr, metric.floatValue);
		} else {
			cbor_encode_int(&entryArr, metric.intValue);
		}
		cbor_encoder_close_container(&valuesArr, &entryArr);
	}

	cbor_encoder_close_container(&topArr, &valuesArr);
	cbor_encoder_close_container(&enc, &topArr);

	size_t used = cbor_encoder_get_buffer_size(&enc, _txBuf.data());
	if (used == 0) return;
	if (_connType == ConnectionType::UDP) {
		_packOrSend(_txBuf.data(), used);
	} else {
		_sendTCP(_txBuf.data(), used);
	}
}

void CborTelemetry::_sendLogCbor(const std::string& message) {
	size_t est = 8 + message.size() + 8;
	if (_txBuf.size() < est) {
		_txBuf.resize(est);
	}

	CborEncoder enc;
	cbor_encoder_init(&enc, _txBuf.data(), _txBuf.size(), 0);

	CborEncoder topArr;
	cbor_encoder_create_array(&enc, &topArr, 2);
	cbor_encode_text_stringz(&topArr, "L");
	cbor_encode_text_string(&topArr, message.c_str(), message.size());
	cbor_encoder_close_container(&enc, &topArr);

	size_t used = cbor_encoder_get_buffer_size(&enc, _txBuf.data());
	if (used == 0) return;
	if (_connType == ConnectionType::UDP) {
		_packOrSend(_txBuf.data(), used);
	} else {
		_sendTCP(_txBuf.data(), used);
	}
}

void CborTelemetry::_ensurePackInit() {
	if (!_packEnable) return;
	if (!_packBuf.empty()) return;
	// Start an indefinite-length array: major type 4, additional 31 => 0x9F
	_packBuf.reserve(_maxUdpPacketSize);
	// Append the '0x9F' byte to _packBuf to signal start of an indefinite-length array in CBOR format
	_packBuf.push_back(0x9F);
}

void CborTelemetry::_flushPackIfNeeded(bool endOfFrame) {
	if (!_packEnable) return;
	if (_packBuf.empty()) return;
	size_t target = _maxUdpPacketSize;
	bool doSend = endOfFrame || (_packBuf.size() >= (target - 1)); // keep room for break
	if (!doSend) return;
	// Close indefinite array with "break": 0xFF
	_packBuf.push_back(0xFF);
	_sendUDP(_packBuf.data(), _packBuf.size());
	_packBuf.clear();
	// Re-initialize for next packet if we expect more data
	if (!endOfFrame) {
		_packBuf.push_back(0x9F);
	}
}

void CborTelemetry::_packOrSend(const uint8_t* data, size_t len) {
	if (!_packEnable || _connType != ConnectionType::UDP) {
		_sendUDP(data, len);
		return;
	}
	_ensurePackInit();
	size_t target = _maxUdpPacketSize;
	// If appending this item would exceed target, flush current buffer first
	if ((_packBuf.size() + len + 1) > target) { // +1 for final break
		_flushPackIfNeeded(true);
		_ensurePackInit();
	}
	_packBuf.insert(_packBuf.end(), data, data + len);
	// If we exactly (or nearly) hit target, flush now
	_flushPackIfNeeded(false);
}



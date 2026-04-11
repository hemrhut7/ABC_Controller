#pragma once
#include <WiFi.h>
#include <WiFiUdp.h>
#include <Arduino.h>

/**
 * @brief UDPStream class that provides a Stream interface for UDP communication.
 * It features "Auto-Targeting" where it remembers the last client that sent a packet
 * and directs all outgoing telemetry data to that client.
 */
class UDPStream : public Stream {
private:
    WiFiUDP _udp;
    IPAddress _remoteIP;
    uint16_t _remotePort = 0; // Target port, initialized to 0 (no target)
    uint16_t _localPort;      // Local port to listen on
    
    uint8_t _tx_buffer[256];  // Buffer to accumulate data before sending as a UDP packet
    size_t _tx_idx = 0;

public:
    UDPStream(uint16_t port) : _localPort(port), _remotePort(0) {}

    /**
     * @brief Initialize the UDP local listener.
     */
    void begin() {
        _udp.begin(_localPort);
    }

    /**
     * @brief Check if a remote target (client) has been identified.
     * @return true if a target IP/Port pair is successfully locked.
     */
    bool connected() const { return _remotePort != 0; }

    // ==========================================
    // Read Interface (Receive Commands)
    // ==========================================

    /**
     * @brief Check if data is available in the current UDP packet.
     * If no packet is active, it attempts to parse the next incoming packet.
     */
    int available() override {
        int len = _udp.available();
        if (len == 0) {
            // No data in current packet, try to parse the next one
            len = _udp.parsePacket();
            if (len > 0) {
                // Auto-Targeting: Update remote IP and Port to lock onto the last sender
                _remoteIP = _udp.remoteIP();
                _remotePort = _udp.remotePort();
                
                Serial.print("[UDP] Telemetry Target Locked -> ");
                Serial.print(_remoteIP);
                Serial.print(":");
                Serial.println(_remotePort);
            }
        }
        return len;
    }

    int read() override { return _udp.read(); }
    int peek() override { return _udp.peek(); }

    // ==========================================
    // Write Interface (Send Telemetry)
    // ==========================================

    /**
     * @brief Writes a single byte to the internal transmit buffer.
     * If a newline '\n' is encountered or the buffer is full, it triggers an immediate flush.
     */
    size_t write(uint8_t c) override {
        if (_remotePort == 0) return 0; // Discard data if no target client is locked
        
        _tx_buffer[_tx_idx++] = c;
        
        if (c == '\n' || _tx_idx >= sizeof(_tx_buffer)) {
            flush();
        }
        return 1;
    }

    /**
     * @brief Writes a block of data directly as a UDP packet.
     */
    size_t write(const uint8_t *buffer, size_t size) override {
        if (_remotePort == 0) return 0;
        _udp.beginPacket(_remoteIP, _remotePort);
        _udp.write(buffer, size);
        _udp.endPacket();
        return size;
    }

    /**
     * @brief Flushes the internal buffer by sending it as a UDP packet.
     */
    void flush() override {
        if (_tx_idx > 0 && _remotePort != 0) {
            _udp.beginPacket(_remoteIP, _remotePort);
            _udp.write(_tx_buffer, _tx_idx);
            _udp.endPacket();
            _tx_idx = 0;
        }
    }
};

#ifndef ENC28J60_HPP
#define ENC28J60_HPP

#include <Arduino.h>
#include <memory>
#include <EthernetESP32.h>
#include <cstdint>
#include <SPI.h>


class ENC28J60 {
public:
  ENC28J60(const uint8_t pinCSEth,
           const uint8_t pinIntEth,
           const uint8_t pinResetEth,
           std::shared_ptr<SPIClass> spi,
           const uint8_t freuencySPI = 3);

  void begin(const IPAddress ipDevice,
              const IPAddress ipGateway,
              const IPAddress subnet,
              const IPAddress ipDns,
              byte macAddress[6],
              const IPAddress ipAgent,
              const uint16_t port = 8888);


  /// @brief Custom transport open callback.
  /// @param transport Required for microros.
  /// @return Success.
  bool udpTransportOpen(struct uxrCustomTransport * transport);


  /// @brief Custom transport close callback.
  /// @param transport Required for microros.
  /// @return Success.
  bool udpTransportClose(struct uxrCustomTransport * transport);


  /// @brief Custom transport write callback.
  /// @param transport Required for microros.
  /// @param buf Required for microros.
  /// @param len Required for microros.
  /// @param err Required for microros.
  /// @return Number of bytes written.
  size_t udpTransportWrite(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err);


  /// @brief Custom transport read callback.
  /// @param transport Required for microros.
  /// @param buf Required for microros.
  /// @param len Required for microros.
  /// @param timeout Required for microros.
  /// @param err Required for microros.
  size_t udpTransportRead(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err);

  
private:
  std::unique_ptr<ENC28J60Driver> eth_;
  std::unique_ptr<NetworkUDP> udp_;
  uint16_t port_;
  IPAddress ipAgent_;
};

#endif // ENC28J60_HPP

#include "ENC28J60.hpp"
#include <freertos/FreeRTOS.h>
#include <esp32-hal-log.h>

ENC28J60::ENC28J60(const uint8_t pinCSEth,
                    const uint8_t pinIntEth,
                    const uint8_t pinResetEth,
                    std::shared_ptr<SPIClass> spi,
                    const uint8_t freuencySPI)
{
  eth_ = std::make_unique<ENC28J60Driver>(pinCSEth, pinIntEth, pinResetEth);
  eth_->setSPI(*spi);
  eth_->setSpiFreq(freuencySPI);
  eth_->usesIRQ();
}


void ENC28J60::begin(const IPAddress ipDevice,
                      const IPAddress ipGateway,
                      const IPAddress subnet,
                      const IPAddress ipDns,
                      byte macAddress[6],
                      const IPAddress ipAgent,
                      const uint16_t port)
{
  port_ = port;
  ipAgent_ = ipAgent;
  Ethernet.init(*eth_);
  Ethernet.begin(macAddress, ipDevice, ipDns, ipGateway, subnet);

  while (!Ethernet.connected()) {
    vTaskDelay(250 / portTICK_PERIOD_MS);
    log_w("ENC28J60: Failed to start");
    Ethernet.begin(macAddress);
  }

  udp_ = std::make_unique<NetworkUDP>();
}


bool  ENC28J60::udpTransportOpen(struct uxrCustomTransport * transport) {
  return udp_->begin(port_);
}


bool  ENC28J60::udpTransportClose(struct uxrCustomTransport * transport) {
  udp_->stop();
  return true;
}


size_t  ENC28J60::udpTransportWrite(struct uxrCustomTransport * transport, const uint8_t * buf, size_t len, uint8_t * err) {
  udp_->beginPacket(ipAgent_, port_);
  size_t written = udp_->write(buf, len);
  return (udp_->endPacket() == 1) ? written : 0;
}


size_t  ENC28J60::udpTransportRead(struct uxrCustomTransport * transport, uint8_t * buf, size_t len, int timeout, uint8_t * err) {
  uint32_t start_time = millis();

  while (udp_->parsePacket() == 0 && (millis() - start_time) < (uint32_t)timeout) {
    delay(1);
  }

  return udp_->read(buf, len);
}
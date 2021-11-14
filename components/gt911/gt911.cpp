#include "esphome/core/log.h"
#include "gt911.h"

namespace esphome {
namespace gt911 {

static const char *TAG = "gt911.sensor";

void GT911::setup(){
    
  this->readBlockData(configBuf, GT911_CONFIG_START, GT911_CONFIG_SIZE);
  this->setResolution(width, height);
}

void GT911::update(){
  this->readTouches();
  this->publish_state(touches);
}

void GT911::dump_config(){

}

void GT911::calculate_checksum() {
  uint8_t checksum;
  for (uint8_t i=0; i<GT911_CONFIG_SIZE; i++) {
    checksum += configBuf[i];
  }
  checksum = (~checksum) + 1;
  configBuf[GT911_CONFIG_CHKSUM - GT911_CONFIG_START] = checksum;
}

void GT911::reflashConfig() {
  this->calculate_checksum();
  this->writeByteData(GT911_CONFIG_CHKSUM, configBuf[GT911_CONFIG_CHKSUM-GT911_CONFIG_START]);
  this->writeByteData(GT911_CONFIG_FRESH, 1);
}

void GT911::setRotation(uint8_t rot) {
  rotation = rot;
}

void GT911::setResolution(uint16_t _width, uint16_t _height) {
  configBuf[GT911_X_OUTPUT_MAX_LOW - GT911_CONFIG_START] = lowByte(_width);
  configBuf[GT911_X_OUTPUT_MAX_HIGH - GT911_CONFIG_START] = highByte(_width);
  configBuf[GT911_Y_OUTPUT_MAX_LOW - GT911_CONFIG_START] = lowByte(_height);
  configBuf[GT911_Y_OUTPUT_MAX_HIGH - GT911_CONFIG_START] = highByte(_height);
  this->reflashConfig();
}
void GT911::readTouches(void) {
  // Serial.println("TAMC_GT911::read");
  uint8_t data[7];
  uint8_t id;
  uint16_t x, y, size;

  uint8_t pointInfo = this->readByteData(GT911_POINT_INFO);
  uint8_t bufferStatus = pointInfo >> 7 & 1;
  uint8_t proximityValid = pointInfo >> 5 & 1;
  uint8_t haveKey = pointInfo >> 4 & 1;
  isLargeDetect = pointInfo >> 6 & 1;
  touches = pointInfo & 0xF;
  // Serial.print("bufferStatus: ");Serial.println(bufferStatus);
  // Serial.print("largeDetect: ");Serial.println(isLargeDetect);
  // Serial.print("proximityValid: ");Serial.println(proximityValid);
  // Serial.print("haveKey: ");Serial.println(haveKey);
  // Serial.print("touches: ");Serial.println(touches);
  isTouched = touches > 0;
  if (bufferStatus == 1 && isTouched) {
    for (uint8_t i=0; i<touches; i++) {
      this->readBlockData(data, GT911_POINT_1 + i * 8, 7);
      points[i] = this->readPoint(data);
    }
  }
  this->writeByteData(GT911_POINT_INFO, 0);
}
TP_Point GT911::readPoint(uint8_t *data) {
  uint16_t temp;
  uint8_t id = data[0];
  uint16_t x = data[1] + (data[2] << 8);
  uint16_t y = data[3] + (data[4] << 8);
  uint16_t size = data[5] + (data[6] << 8);
  switch (rotation){
    case ROTATION_NORMAL:
      x = width - x;
      y = height - y;
      break;
    case ROTATION_LEFT:
      temp = x;
      x = width - y;
      y = temp;
      break;
    case ROTATION_INVERTED:
      x = x;
      y = y;
      break;
    case ROTATION_RIGHT:
      temp = x;
      x = y;
      y = height - temp;
      break;
    default:
      break;
  }
  return TP_Point(id, x, y, size);
}

void GT911::writeByteData(uint16_t reg, uint8_t val) {
  this->write_byte_16(highByte(reg), lowByte(reg) << 8 | val);
}

uint8_t GT911::readByteData(uint16_t reg) {
  uint8_t x;
  this->write_byte(highByte(reg), lowByte(reg));
  uint8_t data;
  this->read(&data, 1);
  return data;
}

void GT911::writeBlockData(uint16_t reg, uint8_t *val, uint8_t size) {
  this->write((uint8_t*)&reg, 2);
  this->write(val, size);
}

void GT911::readBlockData(uint8_t *buf, uint16_t reg, uint8_t size) {
  this->write((uint8_t*)&reg, 2);
  this->read(buf, size);
}

TP_Point::TP_Point(void) {
  id = x = y = size = 0;
}

TP_Point::TP_Point(uint8_t _id, uint16_t _x, uint16_t _y, uint16_t _size) {
  id = _id;
  x = _x;
  y = _y;
  size = _size;
}

bool TP_Point::operator==(TP_Point point) {
  return ((point.x == x) && (point.y == y) && (point.size == size));
}

bool TP_Point::operator!=(TP_Point point) {
  return ((point.x != x) || (point.y != y) || (point.size != size));
}

}  // namespace gt911
}  // namespace esphome
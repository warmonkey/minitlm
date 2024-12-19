#pragma once

class Ublox {
private:
  //char buf[256];
  uint8_t pos;

public:
  Ublox() : pos(0) {

  }

  bool begin() {
    return true;
  }

  void configure() {

  }

  int decode(char c) {
    return 0;
  }

  virtual int parse(const char* buf, size_t len) {
    return 0;
  }
};

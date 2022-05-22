#ifndef __SPI_INTERFACE_H
#define __SPI_INTERFACE_H

template <class DERIVED>
class SPI_Interface {
 public:
  void init() { static_cast<DERIVED*>(this)->init(); }
  void write(uint8_t* data, uint32_t len) {
    static_cast<DERIVED*>(this)->write(data, len);
  }
  void read(uint8_t* data, uint16_t len) {
    static_cast<DERIVED*>(this)->read(data, len);
  }
};

#endif /* __SPI_INTERFACE_H */

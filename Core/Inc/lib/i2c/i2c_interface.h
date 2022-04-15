#ifndef __I2C_Interface
#define __I2C_Interface

template <class DERIVED>
class I2C_Interface {
 public:
  void init() { static_cast<DERIVED*>(this)->init(); }
  void write(uint8_t addr, uint8_t* data, uint16_t len,
             bool repeated_start = false) {
    static_cast<DERIVED*>(this)->write(addr, data, len, repeated_start);
  }
  void read(uint8_t addr, uint8_t* data, uint16_t len,
                   bool stop_cond = false) {
    static_cast<DERIVED*>(this)->read(addr, data, len, stop_cond);
  }
};

#endif /* __I2C_Interface */

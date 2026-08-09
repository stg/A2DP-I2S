#include <stdint.h>
#include <stdbool.h>

class Command {
public:
    Command(const uint8_t *data, size_t length)
        : _data(data), _length(length), _position(0) {}

    // How many bytes left to read
    int available() const {
        return _length - _position;
    }

    // Reset to start
    void rewind() {
        _position = 0;
    }

    // Read a single byte (returns -1 if none left)
    int read() {
        if (_position >= _length) return -1;
        return _data[_position++];
    }

    // --- Convenient typed readers ---
    uint8_t  readU8()  { return (uint8_t)read(); }
    int8_t   readS8()  { return (int8_t)read(); }

    uint16_t readU16() {
        if (available() < 2) return 0;
        uint16_t v = read() | ((uint16_t)read() << 8);
        return v;
    }

    int16_t readS16() {
        return (int16_t)readU16();
    }

    uint32_t readU32() {
        if (available() < 4) return 0;
        uint32_t v = ((uint32_t)read()      ) |
                     ((uint32_t)read() <<  8) |
                     ((uint32_t)read() << 16) |
                     ((uint32_t)read() << 24);
        return v;
    }

    int32_t readS32() {
        return (int32_t)readU32();
    }

private:
    const uint8_t *_data;
    size_t _length;
    size_t _position;
};


class Comm_Driver {
public:
  Comm_Driver() {}
  void init();
  void printf(const char* format, ...);
  int available();
  int read();
  void setCommandHandler(void (*)(Command &));
  void (*getMetaHandler())(uint8_t, const char *, size_t);
  void (*getConnectionHandler())(bool, const char *, size_t);
  void (*getAudioHandler())(int16_t *, bool);
};

extern Comm_Driver Comm;

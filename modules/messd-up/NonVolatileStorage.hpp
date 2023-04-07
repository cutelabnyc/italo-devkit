#ifndef NON_VOLATILE_STORAGE_H
#define NON_VOLATILE_STORAGE_H

#define LFS_MBED_RP2040_VERSION_MIN_TARGET      "LittleFS_Mbed_RP2040 v1.1.0"
#define LFS_MBED_RP2040_VERSION_MIN             1001000

#define _LFS_LOGLEVEL_          4
#define RP2040_FS_SIZE_KB       64

#include <LittleFS_Mbed_RP2040.h>

template <typename DataType>
class NonVolatileStorage {
public:
  NonVolatileStorage();
  uint8_t loadLastPreset(DataType *data);
  uint8_t readPreset(int index, DataType *data);
  uint8_t storePreset(int index, DataType *data);
  template <typename T>
  uint8_t read(const char *path, T *outData);
  template <typename T>
  uint8_t store(const char *path, T *data);

private:
  LittleFS_MBED myFS;
  const char *_indexFilename = "/littlefs/index.txt";
};

#endif // NON_VOLATILE_STORAGE_H

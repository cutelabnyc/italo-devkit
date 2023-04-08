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

template <typename DataType>
NonVolatileStorage<DataType>::NonVolatileStorage() { }

template <typename DataType>
uint8_t NonVolatileStorage<DataType>::loadLastPreset(DataType *data)
{
  Serial.println("Initializing nonvolatlie storage");

  if (!myFS.init()) {
    Serial.println("mount failed");
  }

  Serial.println("Initializing data");
  char buff[512];
  sprintf(buff, "%s", _indexFilename);
  int index;
  uint8_t status = read(buff, &index);

  if (!status) {
    Serial.println("Could not recover index of last saved preset");
    return status;
  }

  sprintf(buff, "/littlefs/preset%d.txt", index);
  status = read(buff, data);

  if (!status) {
    Serial.println("Could not load stored data, using defaults");
  }

  return status;
}

template <typename DataType>
uint8_t NonVolatileStorage<DataType>::readPreset(int index, DataType *data)
{
  Serial.print("Reading preset at index ");
  Serial.println(index);

  char buff[128];
  sprintf(buff, "/littlefs/preset%d.txt", index);
  uint8_t status = read(buff, data);

  // Store the index of the last used file
  if (status) {
    sprintf(buff, "%s", _indexFilename);
    status = store(buff, &index);
  }

  return status;
}

template <typename DataType>
uint8_t NonVolatileStorage<DataType>::storePreset(int index, DataType *data)
{
  Serial.print("Storing preset at index ");
  Serial.println(index);

  char buff[128];
  sprintf(buff, "/littlefs/preset%d.txt", index);
  uint8_t status = store(buff, data);

  // Store the index of the last used file
  if (status) {
    sprintf(buff, "%s", _indexFilename);
    status = store(buff, &index);
  }

  return status;
}

template <typename DataType>
template <typename T>
uint8_t NonVolatileStorage<DataType>::read(const char *path, T *outData)
{
  char buff[512];
  uint8_t status = 0;
  sprintf(buff, "Reading file at %s", path);
  Serial.println(buff);

  FILE *f = fopen(path, "r");

  if (!f) {
    Serial.println("Could not open file for reading");
  } else {
    size_t readBytes = fread(outData, 1, sizeof(T), f);
    if (readBytes >= sizeof(T)) {
      Serial.println("Read data successfully");
      status = 1;
    } else {
      Serial.println("Could not read data file");
    }

    fclose(f);
  }

  return status;
}

template <typename DataType>
template <typename T>
uint8_t NonVolatileStorage<DataType>::store(const char *path, T *data)
{
  char buff[512];
  uint8_t status = 0;
  sprintf(buff, "Storing file at %s", path);
  Serial.println(buff);

  FILE *f = fopen(path, "w");

  if (!f) {
    Serial.println("Could not open file for writing");
  } else {
    size_t writtenBytes = fwrite(data, 1, sizeof(T), f);
    if (writtenBytes >= sizeof(T)) {
      Serial.println("Wrote data successfully");
      status = 1;
    } else {
      Serial.println("Could not write data file");
    }

    fclose(f);
  }

  return status;
}

#endif // NON_VOLATILE_STORAGE_H

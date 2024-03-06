#ifndef STORAGE_H
#define STORAGE_H

#include <string>
#include <Arduino.h>
#include <SPI.h>
#include <SD.h>

// handles writing logs to permanent storage
// Currently implemented using SD card library
// TODO: handle reading config file from SD
// TODO: implement robust error handling and logging
// TODO: implement rtc for timestamping logs?
// TODO: maybe use SdFat for performance?
class Storage {
private:
    uint32_t logs_saved = 0;
    File* current_log;

public:
    Storage();
    ~Storage();
    void new_file();
    void write(const std::string& word);
    void close();
};

#endif // STORAGE_H
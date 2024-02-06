#include "Storage.hpp"

Storage::Storage() {
    if (!SD.begin(BUILTIN_SDCARD)) {
        Serial.println("SD initialization failed");
        return;
    }
}

Storage::~Storage() {
    close();
}

void Storage::new_file() {
    if (!SD.exists("data")) SD.mkdir("data");
    std::string name;
    do {
        name = "data/" + std::to_string(this->logs_saved++) + ".txt";
    } while (SD.exists(name.c_str()));
    
    this->current_log = new File(SD.open(name.c_str(), FILE_WRITE));

    if (! (*current_log)) Serial.println("Failed to open new log file");
}

void Storage::write(const std::string& word) {
    if (current_log != nullptr && *current_log) {
        current_log->print(word.c_str());
    }
    else Serial.println("Problem writing to file");
}

void Storage::close() {
    if (current_log != nullptr && *current_log) {
        current_log->close();
        delete current_log;
        current_log = nullptr;
    }
    else Serial.println("Problem closing log file");
}
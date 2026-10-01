#ifndef PID_CONFIG_HPP
#define PID_CONFIG_HPP

#include <string>
#include <fstream>
#include <iostream>
#include <filesystem>

struct PIDConfig {
    float rollP = 1.0f, rollI = 0.0f, rollD = 0.0f;
    float pitchP = 1.0f, pitchI = 0.0f, pitchD = 0.0f;
    float yawP = 1.0f, yawI = 0.0f, yawD = 0.0f;

    void saveToFile(const std::string& filename) {
        std::filesystem::path p(filename);
        if (p.has_parent_path()) {
            std::filesystem::create_directories(p.parent_path());
        }

        std::ofstream file(filename);
        if (file.is_open()) {
            file << "Roll:  P=" << rollP << " I=" << rollI << " D=" << rollD << "\n";
            file << "Pitch: P=" << pitchP << " I=" << pitchI << " D=" << pitchD << "\n";
            file << "Yaw:   P=" << yawP << " I=" << yawI << " D=" << yawD << "\n";
            file.close();
            std::cout << "[GCS] PID Settings saved to " << filename << std::endl;
        } else {
            std::cerr << "[GCS] Error opening file " << filename << " for saving PID settings.\n";
        }
    }
};

#endif

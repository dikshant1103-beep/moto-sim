#pragma once
// RoadParams — road surface and geometry conditions for the 13-DOF motorcycle simulation.
// Units: SI throughout (m, kg, N, Nm, rad).
// All angles in radians.

#include <string>
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <algorithm>

namespace moto {

struct RoadParams {
    std::string name     = "Dry Asphalt";
    double mu            = 1.0;    // friction scale (1.0 = nominal dry)
    double banking       = 0.0;    // road banking angle [rad], + = right side higher
    double gradient      = 0.0;    // road slope [rad], + = uphill
    double roughness     = 0.0;    // surface roughness RMS [m] (reserved)

    static RoadParams from_yaml(const std::string& path) {
        RoadParams r;
        std::ifstream f(path);
        if (!f.is_open())
            throw std::runtime_error("RoadParams: cannot open file: " + path);

        std::string line;
        while (std::getline(f, line)) {
            // Strip inline comments
            auto hash = line.find('#');
            if (hash != std::string::npos)
                line = line.substr(0, hash);

            // Trim whitespace
            auto ltrim = line.find_first_not_of(" \t\r\n");
            if (ltrim == std::string::npos) continue;
            line = line.substr(ltrim);

            auto colon = line.find(':');
            if (colon == std::string::npos) continue;

            std::string key = line.substr(0, colon);
            std::string val = line.substr(colon + 1);

            // Trim key and val
            auto trim = [](std::string& s) {
                auto a = s.find_first_not_of(" \t\r\n");
                auto b = s.find_last_not_of(" \t\r\n");
                s = (a == std::string::npos) ? "" : s.substr(a, b - a + 1);
            };
            trim(key);
            trim(val);
            if (key.empty() || val.empty()) continue;

            try {
                if (key == "name")       r.name      = val;
                else if (key == "mu")       r.mu        = std::stod(val);
                else if (key == "banking")  r.banking   = std::stod(val);
                else if (key == "gradient") r.gradient  = std::stod(val);
                else if (key == "roughness")r.roughness = std::stod(val);
            } catch (...) {}
        }
        return r;
    }

    static RoadParams dry()  { return {}; }
    static RoadParams wet()  { RoadParams r; r.name="Wet Asphalt";   r.mu=0.60; return r; }
    static RoadParams track(){ RoadParams r; r.name="Race Track";     r.mu=1.10; r.banking=0.04; return r; }
};

} // namespace moto

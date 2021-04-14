/*
 * BSD 2-Clause License
 *
 * Copyright (c) 2020, Christoph Neuhauser
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * * Redistributions of source code must retain the above copyright notice, this
 *   list of conditions and the following disclaimer.
 *
 * * Redistributions in binary form must reproduce the above copyright notice,
 *   this list of conditions and the following disclaimer in the documentation
 *   and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef HEXVOLUMERENDERER_INTERNALSTATE_HPP
#define HEXVOLUMERENDERER_INTERNALSTATE_HPP

#include <string>
#include <map>

#include <glm/glm.hpp>

#include <Utils/Convert.hpp>
#include <Utils/AppSettings.hpp>
#include "Utils/VecStringConversion.hpp"

enum RenderingMode {
    RENDERING_MODE_SURFACE, RENDERING_MODE_WIREFRAME, RENDERING_MODE_DEPTH_COMPLEXITY,
    RENDERING_MODE_CLEAR_VIEW_FACES_UNIFIED, RENDERING_MODE_PSEUDO_VOLUME,
    RENDERING_MODE_VOLUME, RENDERING_MODE_CLEAR_VIEW,
    RENDERING_MODE_VOLUME_FACES, RENDERING_MODE_CLEAR_VIEW_FACES,
    RENDERING_MODE_SINGULARITY, RENDERING_MODE_BASE_COMPLEX_LINES, RENDERING_MODE_BASE_COMPLEX_SURFACE,
    RENDERING_MODE_PARTITION_LINES, RENDERING_MODE_LOD_LINES, RENDERING_MODE_LOD_LINES_PER_FRAGMENT,
    RENDERING_MODE_LOD_LINES_PREVIEW, RENDERING_MODE_LOD_LINES_PREVIEW_SHEETS,
    RENDERING_MODE_SINGULARITY_TYPE_COUNTER, RENDERING_MODE_LINE_DENSITY_CONTROL, RENDERING_MODE_HEX_SHEETS,
    RENDERING_MODE_CLEAR_VIEW_VOLUME2
};
const char *const RENDERING_MODE_NAMES[] = {
        "Surface", "Wireframe", "Depth Complexity",
        "ClearView (Unified)", "Face-Based Volume",
        "Volume", "ClearView (Volume)",
        "Volume (Faces)", "ClearView (Faces)",
        "Singularity", "Base Complex (Lines)", "Base Complex (Surface)",
        "Partition Lines", "LOD Lines", "LOD Lines (Per Frag.)",
        "LOD Lines (Preview)", "LOD Lines (Preview, Sheets)",
        "Singularity Type Counter", "Line Density Control", "Hex Sheets",
        "ClearView (Volume 2)"
};
const int NUM_RENDERING_MODES = ((int)(sizeof(RENDERING_MODE_NAMES) / sizeof(*RENDERING_MODE_NAMES)));

class SettingsMap {
public:
    SettingsMap() {}
    SettingsMap(const std::map<std::string, std::string>& stringMap) : settings(stringMap) {}
    inline std::string getValue(const char *key) const { auto it = settings.find(key); return it == settings.end() ? "" : it->second; }
    inline int getIntValue(const char *key) const { return sgl::fromString<int>(getValue(key)); }
    inline float getFloatValue(const char *key) const { return sgl::fromString<float>(getValue(key)); }
    inline bool getBoolValue(const char *key) const { std::string val = getValue(key); if (val == "false" || val == "0") return false; return val.length() > 0; }
    inline void addKeyValue(const std::string& key, const std::string& value) { settings[key] = value; }
    template<typename T> inline void addKeyValue(const std::string& key, const T& value) { settings[key] = toString(value); }
    inline void clear() { settings.clear(); }
    inline bool hasValue(const char *key) const { auto it = settings.find(key); return it != settings.end(); }

    bool getValueOpt(const char *key, std::string& toset) const {
        auto it = settings.find(key);
        if (it != settings.end()) {
            toset = it->second;
            return true;
        }
        return false;
    }
    bool getValueOpt(const char *key, bool& toset) const {
        auto it = settings.find(key);
        if (it != settings.end()) {
            toset = (it->second == "true") || (it->second == "1");
            return true;
        }
        return false;
    }
    bool getValueOpt(const char *key, glm::vec2& toset) const {
        auto it = settings.find(key);
        if (it != settings.end()) {
            toset = stringToVec2(it->second);
            return true;
        }
        return false;
    }
    bool getValueOpt(const char *key, glm::vec3& toset) const {
        auto it = settings.find(key);
        if (it != settings.end()) {
            toset = stringToVec3(it->second);
            return true;
        }
        return false;
    }
    bool getValueOpt(const char *key, glm::vec4& toset) const {
        auto it = settings.find(key);
        if (it != settings.end()) {
            toset = stringToVec4(it->second);
            return true;
        }
        return false;
    }
    template<typename T> bool getValueOpt(const char *key, T& toset) const {
        auto it = settings.find(key);
        if (it != settings.end()) {
            toset = sgl::fromString<T>(it->second);
            return true;
        }
        return false;
    }

    void set(const std::map<std::string, std::string>& stringMap) {
        settings = stringMap;
    }

    const std::map<std::string, std::string>& getMap() const {
        return settings;
    }

    bool operator==(const SettingsMap& rhs) const {
        return this->settings == rhs.settings;
    }
    bool operator!=(const SettingsMap& rhs) const {
        return !(*this == rhs);
    }

private:
    std::map<std::string, std::string> settings;
};

struct MeshDescriptor {
    MeshDescriptor() {}
    MeshDescriptor(const std::string& paperName, const std::string& meshName, const std::string& meshFileEnding)
            : paperName(paperName), meshName(meshName), meshFileEnding(meshFileEnding) {}
    MeshDescriptor(
            const std::string& paperName, const std::string& meshName,
            const std::string& meshFileEnding, float deformation)
            : paperName(paperName), meshName(meshName), meshFileEnding(meshFileEnding), deformation(deformation) {}

    bool operator==(const MeshDescriptor& rhs) const {
        return this->paperName == rhs.paperName && this->meshName == rhs.meshName
                && this->meshFileEnding == rhs.meshFileEnding;
    }
    bool operator!=(const MeshDescriptor& rhs) const {
        return !(*this == rhs);
    }

    std::string getFilename() const {
        return sgl::AppSettings::get()->getDataDirectory()
                + "Meshes/" + paperName + "/" + meshName + "." + meshFileEnding;
    }

    std::string paperName;
    std::string meshName;
    std::string meshFileEnding;
    float deformation = 0.0f; ///< For deformed meshes only.
};

struct InternalState {
    bool operator==(const InternalState& rhs) const {
        return this->meshDescriptor == rhs.meshDescriptor && this->name == rhs.name
                && this->renderingMode == rhs.renderingMode
                && this->rendererSettings == rhs.rendererSettings
                && this->filterSettings == rhs.filterSettings
                && this->tilingWidth == rhs.tilingWidth && this->tilingHeight == rhs.tilingHeight
                && this->useMortonCodeForTiling == rhs.useMortonCodeForTiling
                && this->transferFunctionName == rhs.transferFunctionName
                && this->windowResolution == rhs.windowResolution;
    }

    bool operator!=(const InternalState& rhs) const {
        return !(*this == rhs);
    }

    MeshDescriptor meshDescriptor;
    std::string name;
    RenderingMode renderingMode;
    std::vector<SettingsMap> rendererSettings;
    std::vector<SettingsMap> filterSettings;
    int tilingWidth = 2;
    int tilingHeight = 8;
    bool useMortonCodeForTiling = false;
    std::string transferFunctionName;
    glm::ivec2 windowResolution = glm::ivec2(0, 0);
};

std::vector<InternalState> getTestModesPaper();
std::vector<InternalState> getTestModesSorting();

#endif //HEXVOLUMERENDERER_INTERNALSTATE_HPP

/**
 * @file JsonLoader.cpp
 * @brief JSON loader implementation with simple parser
 * @copyright Defense-grade radar tracking system
 */

#include "zoppler/config/JsonLoader.hpp"
#include <fstream>
#include <sstream>
#include <stdexcept>
#include <cctype>
#include <map>
#include <vector>
#include <variant>

namespace zoppler {

namespace {  // Anonymous namespace for implementation details

// Simple JSON value type
struct JsonValue {
    using Object = std::map<std::string, JsonValue>;
    using Array = std::vector<JsonValue>;
    using Value = std::variant<std::monostate, bool, double, std::string, Array, Object>;
    
    Value value;
    
    JsonValue() : value(std::monostate{}) {}
    JsonValue(bool b) : value(b) {}
    JsonValue(double d) : value(d) {}
    JsonValue(const std::string& s) : value(s) {}
    JsonValue(Array a) : value(std::move(a)) {}
    JsonValue(Object o) : value(std::move(o)) {}
    
    bool isNull() const { return std::holds_alternative<std::monostate>(value); }
    bool isBool() const { return std::holds_alternative<bool>(value); }
    bool isNumber() const { return std::holds_alternative<double>(value); }
    bool isString() const { return std::holds_alternative<std::string>(value); }
    bool isArray() const { return std::holds_alternative<Array>(value); }
    bool isObject() const { return std::holds_alternative<Object>(value); }
    
    bool getBool() const { return std::get<bool>(value); }
    double getNumber() const { return std::get<double>(value); }
    const std::string& getString() const { return std::get<std::string>(value); }
    const Array& getArray() const { return std::get<Array>(value); }
    const Object& getObject() const { return std::get<Object>(value); }
    
    bool has(const std::string& key) const {
        if (!isObject()) return false;
        return getObject().find(key) != getObject().end();
    }
    
    const JsonValue& operator[](const std::string& key) const {
        static JsonValue null;
        if (!isObject()) return null;
        auto it = getObject().find(key);
        return (it != getObject().end()) ? it->second : null;
    }
    
    double getNumberOr(double def) const {
        return isNumber() ? getNumber() : def;
    }
    
    bool getBoolOr(bool def) const {
        return isBool() ? getBool() : def;
    }
    
    std::string getStringOr(const std::string& def) const {
        return isString() ? getString() : def;
    }
};

// Simple JSON parser
class JsonParser {
public:
    explicit JsonParser(const std::string& json) : json_(json), pos_(0) {}
    
    JsonValue parse() {
        skipWhitespace();
        return parseValue();
    }
    
private:
    const std::string& json_;
    size_t pos_;
    
    void skipWhitespace() {
        while (pos_ < json_.size() && std::isspace(json_[pos_])) {
            ++pos_;
        }
    }
    
    char peek() const {
        return (pos_ < json_.size()) ? json_[pos_] : '\0';
    }
    
    char consume() {
        return (pos_ < json_.size()) ? json_[pos_++] : '\0';
    }
    
    void expect(char c) {
        skipWhitespace();
        if (consume() != c) {
            throw std::runtime_error("JSON parse error: expected '" + std::string(1, c) + "'");
        }
    }
    
    JsonValue parseValue() {
        skipWhitespace();
        char c = peek();
        
        if (c == '"') return parseString();
        if (c == '{') return parseObject();
        if (c == '[') return parseArray();
        if (c == 't' || c == 'f') return parseBool();
        if (c == 'n') return parseNull();
        if (c == '-' || std::isdigit(c)) return parseNumber();
        
        throw std::runtime_error("JSON parse error: unexpected character");
    }
    
    JsonValue parseString() {
        expect('"');
        std::string result;
        
        while (pos_ < json_.size() && json_[pos_] != '"') {
            if (json_[pos_] == '\\') {
                ++pos_;
                if (pos_ >= json_.size()) break;
                char escaped = json_[pos_++];
                switch (escaped) {
                    case 'n': result += '\n'; break;
                    case 't': result += '\t'; break;
                    case 'r': result += '\r'; break;
                    case '"': result += '"'; break;
                    case '\\': result += '\\'; break;
                    default: result += escaped; break;
                }
            } else {
                result += json_[pos_++];
            }
        }
        
        expect('"');
        return JsonValue(result);
    }
    
    JsonValue parseNumber() {
        size_t start = pos_;
        
        if (json_[pos_] == '-') ++pos_;
        
        while (pos_ < json_.size() && std::isdigit(json_[pos_])) ++pos_;
        
        if (pos_ < json_.size() && json_[pos_] == '.') {
            ++pos_;
            while (pos_ < json_.size() && std::isdigit(json_[pos_])) ++pos_;
        }
        
        if (pos_ < json_.size() && (json_[pos_] == 'e' || json_[pos_] == 'E')) {
            ++pos_;
            if (pos_ < json_.size() && (json_[pos_] == '+' || json_[pos_] == '-')) ++pos_;
            while (pos_ < json_.size() && std::isdigit(json_[pos_])) ++pos_;
        }
        
        std::string numStr = json_.substr(start, pos_ - start);
        return JsonValue(std::stod(numStr));
    }
    
    JsonValue parseBool() {
        if (json_.compare(pos_, 4, "true") == 0) {
            pos_ += 4;
            return JsonValue(true);
        }
        if (json_.compare(pos_, 5, "false") == 0) {
            pos_ += 5;
            return JsonValue(false);
        }
        throw std::runtime_error("JSON parse error: expected boolean");
    }
    
    JsonValue parseNull() {
        if (json_.compare(pos_, 4, "null") == 0) {
            pos_ += 4;
            return JsonValue();
        }
        throw std::runtime_error("JSON parse error: expected null");
    }
    
    JsonValue parseArray() {
        expect('[');
        JsonValue::Array arr;
        
        skipWhitespace();
        if (peek() == ']') {
            consume();
            return JsonValue(arr);
        }
        
        while (true) {
            arr.push_back(parseValue());
            
            skipWhitespace();
            if (peek() == ']') {
                consume();
                break;
            }
            expect(',');
        }
        
        return JsonValue(arr);
    }
    
    JsonValue parseObject() {
        expect('{');
        JsonValue::Object obj;
        
        skipWhitespace();
        if (peek() == '}') {
            consume();
            return JsonValue(obj);
        }
        
        while (true) {
            skipWhitespace();
            auto keyValue = parseString();
            std::string key = keyValue.getString();
            
            skipWhitespace();
            expect(':');
            
            obj[key] = parseValue();
            
            skipWhitespace();
            if (peek() == '}') {
                consume();
                break;
            }
            expect(',');
        }
        
        return JsonValue(obj);
    }
};

JsonValue parseJson(const std::string& json) {
    JsonParser parser(json);
    return parser.parse();
}

TrackerConfig convertToConfig(const JsonValue& root) {
    TrackerConfig config;
    
    if (!root.isObject()) {
        throw std::runtime_error("JSON root must be an object");
    }
    
    // Parse radar configuration
    if (root.has("radar")) {
        const auto& radar = root["radar"];
        if (radar.isObject()) {
            config.radar.is3D = radar["is3D"].getBoolOr(true);
            config.radar.hasDoppler = radar["hasDoppler"].getBoolOr(true);
            config.radar.updateTime = radar["updateTime"].getNumberOr(0.1);
            
            // Accuracy
            if (radar.has("accuracy")) {
                const auto& acc = radar["accuracy"];
                config.radar.rangeStd = acc["range"].getNumberOr(1.0);
                config.radar.azimuthStd = acc["azimuth"].getNumberOr(0.01);
                config.radar.elevationStd = acc["elevation"].getNumberOr(0.02);
                config.radar.dopplerStd = acc["doppler"].getNumberOr(0.5);
            }
            
            // Coverage
            if (radar.has("coverage")) {
                const auto& cov = radar["coverage"];
                config.radar.maxRange = cov["maxRange"].getNumberOr(50000.0);
                config.radar.minRange = cov["minRange"].getNumberOr(50.0);
            }
            
            config.radar.pD = radar["pD"].getNumberOr(0.9);
        }
    }
    
    // Parse target type
    if (root.has("targetType")) {
        config.targetType = root["targetType"].getStringOr("DRONE");
    }
    
    // Parse clustering
    if (root.has("clustering")) {
        const auto& clust = root["clustering"];
        if (clust.isObject()) {
            config.clusteringAlgo = clust["algorithm"].getStringOr("DBSCAN");
            
            if (clust.has("epsilon")) {
                config.clusteringParams.epsilon = clust["epsilon"].getNumberOr(10.0);
            }
            if (clust.has("minPoints")) {
                config.clusteringParams.minPoints = static_cast<int>(clust["minPoints"].getNumberOr(2));
            }
        }
    }
    
    // Parse association
    if (root.has("association")) {
        const auto& assoc = root["association"];
        if (assoc.isObject()) {
            config.associationAlgo = assoc["algorithm"].getStringOr("GNN");
            
            if (assoc.has("maxDistance")) {
                config.associationParams.maxAssociationDistance = assoc["maxDistance"].getNumberOr(100.0);
            }
            if (assoc.has("pD")) {
                config.associationParams.jpdaPD = assoc["pD"].getNumberOr(0.9);
            }
        }
    }
    
    // Parse tracking
    if (root.has("tracking")) {
        const auto& track = root["tracking"];
        if (track.isObject()) {
            config.trackingAlgo = track["algorithm"].getStringOr("EKF");
            config.motionModel = track["motionModel"].getStringOr("CV");
            config.profile = track["profile"].getStringOr("SHORT_RANGE");
            
            // UKF parameters
            if (track.has("ukfAlpha")) {
                config.trackingParams.ukfAlpha = track["ukfAlpha"].getNumberOr(0.001);
            }
            
            // Particle filter parameters
            if (track.has("numParticles")) {
                config.trackingParams.pfNumParticles = static_cast<int>(track["numParticles"].getNumberOr(500));
            }
            
            // IMM models
            if (track.has("immModels") && track["immModels"].isArray()) {
                config.trackingParams.immModels.clear();
                for (const auto& model : track["immModels"].getArray()) {
                    if (model.isString()) {
                        config.trackingParams.immModels.push_back(model.getString());
                    }
                }
            }
        }
    }
    
    // Parse track management
    if (root.has("trackManagement")) {
        const auto& tm = root["trackManagement"];
        if (tm.isObject()) {
            config.trackManagement.minHitsToConfirm = static_cast<int>(tm["minHits"].getNumberOr(3));
            config.trackManagement.confirmWindow = static_cast<int>(tm["confirmWindow"].getNumberOr(5));
            config.trackManagement.maxConsecutiveMisses = static_cast<int>(tm["maxMisses"].getNumberOr(5));
            config.trackManagement.maxTracks = static_cast<int>(tm["maxTracks"].getNumberOr(500));
        }
    }
    
    // Resolve profile
    config.resolveProfile();
    
    return config;
}

}  // anonymous namespace

TrackerConfig JsonLoader::loadFromFile(const std::string& filepath) {
    std::ifstream file(filepath);
    if (!file.is_open()) {
        throw std::runtime_error("Could not open file: " + filepath);
    }
    
    std::stringstream buffer;
    buffer << file.rdbuf();
    
    return loadFromString(buffer.str());
}

TrackerConfig JsonLoader::loadFromString(const std::string& jsonString) {
    auto root = parseJson(jsonString);
    return convertToConfig(root);
}

} // namespace zoppler

#ifndef LEG_KILO_YAML_HELPER_H
#define LEG_KILO_YAML_HELPER_H

#include <glog/logging.h>
#include <yaml-cpp/yaml.h>
#include <iostream>
#include <sstream>
#include <string>

template <typename T>
std::ostream& operator<<(std::ostream& os, const std::vector<T>& vec) {
    os << "[";
    for (size_t i = 0; i < vec.size(); ++i) {
        os << vec[i];
        if (i != vec.size() - 1) os << ", ";
    }
    os << "]";
    return os;
}

namespace legkilo {

inline std::string normalizeRootDir(const std::string& root_dir) {
    if (root_dir.empty()) { return std::string(ROOT_DIR); }
    if (root_dir.back() == '/') { return root_dir; }
    return root_dir + "/";
}

inline std::string defaultRootDir() { return normalizeRootDir(std::string(ROOT_DIR)); }

class YamlHelper {
   public:
    YamlHelper() = delete;

    explicit YamlHelper(const std::string& config_file) {
        try {
            yaml_node_ = YAML::LoadFile(config_file);
        } catch (const std::exception& e) {
            LOG(ERROR) << "Failed to open YAML file: " << config_file << "Errors: " << e.what();
            throw std::runtime_error("Failed to open YAML file: " + config_file);
        }
    }

    bool hasKey(const std::string& key) const { return yaml_node_[key] ? true : false; }

    template <typename T>
    T get(const std::string& key) const {
        if (!hasKey(key)) {
            LOG(ERROR) << "Failed to find key: " << key;
            throw std::runtime_error("Failed to find key: " + key);
        }
        try {
            T ret = yaml_node_[key].as<T>();
            LOG(INFO) << "YAML Key:  " << key << " = " << ret;
            return ret;
        } catch (const std::exception& e) {
            LOG(ERROR) << "Failed to convert key " << key << "Errors: " << e.what();
            throw std::runtime_error("Failed to convert key " + key);
        }
    }

    template <typename T>
    T get(const std::string& key, const T& default_value) const {
        if (!hasKey(key)) {
            LOG(WARNING) << "Key not found: " << key << ", returning default: " << default_value;
            return default_value;
        }
        try {
            T ret = yaml_node_[key].as<T>();
            LOG(INFO) << "YAML Key: " << key << " = " << ret;
            return ret;
        } catch (const std::exception& e) {
            LOG(WARNING) << "Failed to convert key " << key << ", returning default: " << default_value
                         << ". Error: " << e.what();
            return default_value;
        }
    }

   private:
    YAML::Node yaml_node_;
};

inline std::string resolveRootDir(const YamlHelper& yaml_helper) {
    if (!yaml_helper.hasKey("root_dir")) { return defaultRootDir(); }

    const std::string configured_root_dir = yaml_helper.get<std::string>("root_dir", "");
    if (configured_root_dir.empty()) {
        LOG(WARNING) << "Key root_dir is empty, fallback to default: " << defaultRootDir();
        return defaultRootDir();
    }

    return normalizeRootDir(configured_root_dir);
}

inline std::string resolveRootDirFromConfigFile(const std::string& config_file) {
    try {
        const YAML::Node yaml_node = YAML::LoadFile(config_file);
        const YAML::Node root_dir_node = yaml_node["root_dir"];
        if (!root_dir_node || !root_dir_node.IsScalar()) { return defaultRootDir(); }

        const std::string configured_root_dir = root_dir_node.as<std::string>();
        if (configured_root_dir.empty()) { return defaultRootDir(); }
        return normalizeRootDir(configured_root_dir);
    } catch (const std::exception&) {
        return defaultRootDir();
    }
}

}  // namespace legkilo
#endif  // LEG_KILO_YAML_HELPER_H
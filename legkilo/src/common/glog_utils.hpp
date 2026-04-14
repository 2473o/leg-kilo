#ifndef LEG_KILO_GLOG_UTILS_H
#define LEG_KILO_GLOG_UTILS_H

#include <iostream>
#include <string>

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <boost/filesystem.hpp>

#include "common/yaml_helper.hpp"

namespace fs = boost::filesystem;

namespace legkilo {

class Logging {
   public:
    Logging(int argc, char** argv, const std::string& log_dir, const std::string& root_dir = defaultRootDir());
    ~Logging();
    bool createLogFile(const std::string& dir);
    void flushLogFiles();
};

inline Logging::Logging(int argc, char** argv, const std::string& log_dir, const std::string& root_dir) {
    (void)argc;
    const std::string full_log_dir = root_dir + log_dir;
    if (!createLogFile(full_log_dir)) { throw std::runtime_error("Create Log File Failed"); }

    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    
    // 关闭 glog 的默认 30s 缓冲机制，强制立即刷新每一条日志到终端
    FLAGS_logbufsecs = 0;

    google::InitGoogleLogging(argv[0]);
    
    // 注册失败信号处理器，使得崩溃时也能打出最后一条完整日志
    google::InstallFailureSignalHandler();

    FLAGS_log_dir = full_log_dir;

    std::cout << "\033[33m"
              << "GLOG ON"
              << "\033[0m" << std::endl;
}

inline Logging::~Logging() {
    google::ShutdownGoogleLogging();
    std::cout << "\033[33m"
              << "GLOG OFF"
              << "\033[0m" << std::endl;
}

inline bool Logging::createLogFile(const std::string& dir) {
    if (!fs::exists(dir)) {
        std::cout << "Creating Log File" << std::endl;
        try {
            if (fs::create_directory(dir)) {
                std::cout << "Folder created successfully" << std::endl;
                return true;
            } else {
                std::cerr << "Failed to create folder" << std::endl;
                return false;
            }
        } catch (const fs::filesystem_error& e) {
            std::cerr << "Error: " << e.what() << std::endl;
            return false;
        }
    }

    return true;
}

inline void Logging::flushLogFiles() {
    google::FlushLogFiles(google::INFO);
    google::FlushLogFiles(google::WARNING);
    google::FlushLogFiles(google::ERROR);
}

}  // namespace legkilo
#endif  // LEG_KILO_GLOG_UTILS_H

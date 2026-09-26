#pragma once

#include <iostream>
#include <string>
#include <vector>
#include <unordered_map>

class ArgumentParser {
public:
    ArgumentParser(int argc, char** argv) {
        for (int i = 1; i < argc; ++i) {
            std::string arg = argv[i];
            if (arg.compare(0, 2, "--") == 0) {  // Long options
                size_t pos = arg.find('=');
                std::string key = (pos != std::string::npos) ? arg.substr(2, pos - 2) : arg.substr(2);
                std::string value = (pos != std::string::npos) ? arg.substr(pos + 1) : "";
                args_[key] = value;
            } else if (arg.compare(0, 1, "-") == 0) {  // Short options
                for (size_t j = 1; j < arg.size(); ++j) {
                    std::string key(1, arg[j]);
                    std::string value;
                    if (j + 1 < arg.size() && arg[j + 1] == '=') {
                        value = arg.substr(j + 2);
                        args_[key] = value;
                        break;
                    } else {
                        args_[key] = "";
                    }
                }
            }
        }
    }

    bool has(const std::string& key) const {
        return args_.find(key) != args_.end();
    }

    std::string get(const std::string& key) const {
        if (has(key)) {
            return args_.at(key);
        }
        return "";
    }

    void printUsage() const {
        std::cout << "Usage:\n";
        for (const auto& [key, value] : args_) {
            std::cout << "  --" << key;
            if (!value.empty()) {
                std::cout << " = " << value;
            }
            std::cout << '\n';
        }
    }

private:
    std::unordered_map<std::string, std::string> args_;
};

// int main(int argc, char** argv) {
//     ArgumentParser parser(argc, argv);

//     // Check for specific arguments
//     if (parser.has("help")) {
//         parser.printUsage();
//         return 0;
//     }

//     if (parser.has("name")) {
//         std::cout << "Hello, " << parser.get("name") << "!\n";
//     } else {
//         std::cout << "Hello, World!\n";
//     }

//     return 0;
// }
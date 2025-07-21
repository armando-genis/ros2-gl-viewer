#include <fstream>
#include <sstream>
#include <string>


private:
    size_t gpu_memory_used_mb_ = 0;
    size_t gpu_memory_total_mb_ = 0;
    float gpu_utilization_percent_ = 0.0f;
    std::string gpu_name_ = "Unknown GPU";
    bool gpu_available_ = false;

// Add this method to your Ros2GLViewer class:
void checkGPUUsage()
{
    // Reset values
    gpu_memory_used_mb_ = 0;
    gpu_memory_total_mb_ = 0;
    gpu_utilization_percent_ = 0.0f;
    gpu_available_ = false;

    // Try NVIDIA first
    if (checkNVIDIAGPU()) {
        gpu_available_ = true;
        return;
    }
    
    // Try AMD/Intel
    if (checkAMDIntelGPU()) {
        gpu_available_ = true;
        return;
    }
}

bool checkNVIDIAGPU()
{
    // Get GPU name, memory usage, and utilization in one call
    std::string command = "nvidia-smi --query-gpu=name,memory.used,memory.total,utilization.gpu --format=csv,noheader,nounits 2>/dev/null";
    
    FILE* pipe = popen(command.c_str(), "r");
    if (!pipe) return false;
    
    char buffer[512];
    std::string result;
    if (fgets(buffer, sizeof(buffer), pipe) != nullptr) {
        result = buffer;
    }
    pclose(pipe);
    
    if (result.empty()) return false;
    
    // Parse: "GPU Name, memory_used_mb, memory_total_mb, utilization_percent"
    std::istringstream iss(result);
    std::string token;
    std::vector<std::string> tokens;
    
    while (std::getline(iss, token, ',')) {
        // Trim whitespace
        token.erase(0, token.find_first_not_of(" \t"));
        token.erase(token.find_last_not_of(" \t\n\r") + 1);
        tokens.push_back(token);
    }
    
    if (tokens.size() >= 4) {
        gpu_name_ = tokens[0];
        gpu_memory_used_mb_ = std::stoul(tokens[1]);
        gpu_memory_total_mb_ = std::stoul(tokens[2]);
        gpu_utilization_percent_ = std::stof(tokens[3]);
        return true;
    }
    
    return false;
}

bool checkAMDIntelGPU()
{
    // Try to find GPU info from /sys/class/drm/
    for (int card = 0; card < 4; ++card) {
        std::string card_path = "/sys/class/drm/card" + std::to_string(card);
        
        // Check if this card exists
        std::ifstream test(card_path + "/device/vendor");
        if (!test.is_open()) continue;
        
        // Try to get memory info (AMD style)
        std::string vram_used_path = card_path + "/device/mem_info_vram_used";
        std::string vram_total_path = card_path + "/device/mem_info_vram_total";
        
        std::ifstream used_file(vram_used_path);
        std::ifstream total_file(vram_total_path);
        
        if (used_file.is_open() && total_file.is_open()) {
            std::string used_str, total_str;
            std::getline(used_file, used_str);
            std::getline(total_file, total_str);
            
            try {
                // Convert from bytes to MB
                gpu_memory_used_mb_ = std::stoull(used_str) / (1024 * 1024);
                gpu_memory_total_mb_ = std::stoull(total_str) / (1024 * 1024);
            } catch (...) {
                continue;
            }
        }
        
        // Try to get GPU utilization
        std::string busy_path = card_path + "/device/gpu_busy_percent";
        std::ifstream busy_file(busy_path);
        if (busy_file.is_open()) {
            std::string busy_str;
            std::getline(busy_file, busy_str);
            try {
                gpu_utilization_percent_ = std::stof(busy_str);
            } catch (...) {
                gpu_utilization_percent_ = 0.0f;
            }
        }
        
        // Try to get GPU name from device
        std::string name_path = card_path + "/device/device";
        std::ifstream name_file(name_path);
        if (name_file.is_open()) {
            std::string device_id;
            std::getline(name_file, device_id);
            gpu_name_ = "GPU Card " + std::to_string(card) + " (" + device_id + ")";
        } else {
            gpu_name_ = "GPU Card " + std::to_string(card);
        }
        
        return true; // Found at least some GPU info
    }
    
    return false;
}

// Modify your draw_ui() method in the memory monitoring section:
void draw_ui() override
{
    // ... existing code until memory monitoring section ...

    // ✅ Memory and GPU monitoring
    static int frame_count = 0;
    frame_count++;
    if (frame_count % 60 == 0) { // Check every ~1 second at 60fps
        checkMemoryUsage();
        checkGPUUsage();
    }
    
    ImGui::Text("CPU Memory: %zu MB", last_memory_check_);
    
    // GPU information (btop style)
    if (gpu_available_) {
        ImGui::Text("GPU: %s", gpu_name_.c_str());
        ImGui::Text("GPU Usage: %.1f%% | VRAM: %zu/%zu MB", 
                   gpu_utilization_percent_, 
                   gpu_memory_used_mb_, 
                   gpu_memory_total_mb_);
    } else {
        ImGui::Text("GPU: Not detected");
    }

    // ... rest of existing code ...
}
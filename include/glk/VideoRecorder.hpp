#pragma once

#include <string>
#include <memory>
#include <vector>
#include <mutex>
#include <thread>
#include <atomic>
#include <Eigen/Core>

extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/avutil.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
#include <libswscale/swscale.h>
}

class VideoRecorder {
public:
    VideoRecorder();
    ~VideoRecorder();

    // Start recording to file
    // quality: CRF value (18 = high quality, 23 = medium, 28 = lower quality)
    bool startRecording(const std::string& filename, int width, int height, int fps = 30, int quality = 18);
    
    // Stop recording and finalize file
    void stopRecording();
    
    // Add a frame from OpenGL framebuffer (RGB format)
    // rgb_data should be RGB24 format with stride = input_width * 3
    // If input_width/height are 0, assumes they match recording dimensions
    bool addFrame(const uint8_t* rgb_data, int input_width = 0, int input_height = 0);
    
    // Check if currently recording
    bool isRecording() const { return is_recording_; }
    
    // Get current frame count
    size_t getFrameCount() const { return frame_count_; }

private:
    bool initializeCodec(int width, int height, int fps, int quality);
    void cleanup();
    bool encodeFrame(const uint8_t* rgb_data, int input_width, int input_height);
    
    std::string filename_;
    int width_;
    int height_;
    int fps_;
    int quality_;
    
    AVFormatContext* format_context_;
    AVCodecContext* codec_context_;
    AVStream* video_stream_;
    AVFrame* frame_;
    AVPacket* packet_;
    SwsContext* sws_context_;
    
    uint8_t* frame_buffer_;
    
    std::atomic<bool> is_recording_;
    std::atomic<size_t> frame_count_;
    
    std::mutex recording_mutex_;
};


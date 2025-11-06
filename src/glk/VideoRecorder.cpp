#include "glk/VideoRecorder.hpp"
#include <iostream>
#include <stdexcept>

VideoRecorder::VideoRecorder()
    : format_context_(nullptr)
    , codec_context_(nullptr)
    , video_stream_(nullptr)
    , frame_(nullptr)
    , packet_(nullptr)
    , sws_context_(nullptr)
    , frame_buffer_(nullptr)
    , is_recording_(false)
    , frame_count_(0)
    , width_(0)
    , height_(0)
    , fps_(30)
{
}

VideoRecorder::~VideoRecorder()
{
    stopRecording();
}

bool VideoRecorder::startRecording(const std::string& filename, int width, int height, int fps, int quality)
{
    std::lock_guard<std::mutex> lock(recording_mutex_);
    
    if (is_recording_) {
        std::cerr << "Already recording!" << std::endl;
        return false;
    }
    
    // H.264 requires dimensions to be divisible by 2 (for YUV420P format)
    // Round down to nearest even number
    width_ = (width / 2) * 2;
    height_ = (height / 2) * 2;
    
    if (width_ != width || height_ != height) {
        std::cout << "⚠️ Adjusting video dimensions from " << width << "x" << height 
                  << " to " << width_ << "x" << height_ << " (must be even for H.264)" << std::endl;
    }
    
    filename_ = filename;
    fps_ = fps;
    quality_ = quality; // Store quality setting
    frame_count_ = 0;
    
    if (!initializeCodec(width_, height_, fps, quality)) {
        std::cerr << "Failed to initialize codec" << std::endl;
        return false;
    }
    
    is_recording_ = true;
    std::cout << "✅ Started recording to: " << filename_ 
              << " (" << width_ << "x" << height_ << "@" << fps_ << "fps, CRF=" << quality_ << ")" << std::endl;
    
    return true;
}

void VideoRecorder::stopRecording()
{
    std::lock_guard<std::mutex> lock(recording_mutex_);
    
    if (!is_recording_) {
        return;
    }
    
    is_recording_ = false;
    
    // Flush encoder
    if (codec_context_) {
        encodeFrame(nullptr, 0, 0); // Send nullptr to flush
    }
    
    // Write trailer
    if (format_context_) {
        av_write_trailer(format_context_);
    }
    
    cleanup();
    
    std::cout << "✅ Stopped recording. Total frames: " << frame_count_ 
              << " -> " << filename_ << std::endl;
}

bool VideoRecorder::initializeCodec(int width, int height, int fps, int quality)
{
    int ret;
    
    // Allocate format context
    ret = avformat_alloc_output_context2(&format_context_, nullptr, nullptr, filename_.c_str());
    if (ret < 0) {
        std::cerr << "Could not allocate output context: " << ret << std::endl;
        return false;
    }
    
    // Find H.264 encoder
    const AVCodec* codec = avcodec_find_encoder(AV_CODEC_ID_H264);
    if (!codec) {
        std::cerr << "H.264 codec not found" << std::endl;
        return false;
    }
    
    // Create video stream
    video_stream_ = avformat_new_stream(format_context_, codec);
    if (!video_stream_) {
        std::cerr << "Could not create video stream" << std::endl;
        return false;
    }
    
    // Allocate codec context
    codec_context_ = avcodec_alloc_context3(codec);
    if (!codec_context_) {
        std::cerr << "Could not allocate codec context" << std::endl;
        return false;
    }
    
    // Set codec parameters
    codec_context_->codec_id = codec->id;
    codec_context_->codec_type = AVMEDIA_TYPE_VIDEO;
    codec_context_->pix_fmt = AV_PIX_FMT_YUV420P;
    codec_context_->width = width;
    codec_context_->height = height;
    codec_context_->time_base = {1, fps};
    codec_context_->framerate = {fps, 1};
    codec_context_->gop_size = 12; // Group of pictures
    codec_context_->max_b_frames = 1;
    
    // Set codec options for better quality (if supported by codec)
    if (codec_context_->priv_data) {
        // Use "slow" preset for better quality (uses more CPU but produces better compression)
        // Options: ultrafast, superfast, veryfast, faster, fast, medium, slow, slower, veryslow
        int ret_preset = av_opt_set(codec_context_->priv_data, "preset", "slow", 0);
        
        // CRF (Constant Rate Factor): lower = better quality, larger file
        // 18 = nearly lossless, 23 = high quality (default), 28 = lower quality
        // Use the quality parameter passed in
        char crf_str[16];
        snprintf(crf_str, sizeof(crf_str), "%d", quality);
        int ret_crf = av_opt_set(codec_context_->priv_data, "crf", crf_str, 0);
        
        // Ignore errors if options are not supported by this codec
        (void)ret_preset;
        (void)ret_crf;
    }
    
    // Open codec
    ret = avcodec_open2(codec_context_, codec, nullptr);
    if (ret < 0) {
        std::cerr << "Could not open codec: " << ret << std::endl;
        return false;
    }
    
    // Copy codec parameters to stream
    ret = avcodec_parameters_from_context(video_stream_->codecpar, codec_context_);
    if (ret < 0) {
        std::cerr << "Could not copy codec parameters: " << ret << std::endl;
        return false;
    }
    
    video_stream_->time_base = codec_context_->time_base;
    
    // Open output file
    if (!(format_context_->oformat->flags & AVFMT_NOFILE)) {
        ret = avio_open(&format_context_->pb, filename_.c_str(), AVIO_FLAG_WRITE);
        if (ret < 0) {
            std::cerr << "Could not open output file: " << ret << std::endl;
            return false;
        }
    }
    
    // Allocate frame
    frame_ = av_frame_alloc();
    if (!frame_) {
        std::cerr << "Could not allocate frame" << std::endl;
        return false;
    }
    
    frame_->format = codec_context_->pix_fmt;
    frame_->width = codec_context_->width;
    frame_->height = codec_context_->height;
    
    ret = av_frame_get_buffer(frame_, 32);
    if (ret < 0) {
        std::cerr << "Could not allocate frame buffer: " << ret << std::endl;
        return false;
    }
    
    // Allocate packet
    packet_ = av_packet_alloc();
    if (!packet_) {
        std::cerr << "Could not allocate packet" << std::endl;
        return false;
    }
    
    // Note: width and height are already guaranteed to be even at this point
    // Initialize SWS context for RGB to YUV conversion
    sws_context_ = sws_getContext(
        width, height, AV_PIX_FMT_RGB24,
        width, height, AV_PIX_FMT_YUV420P,
        SWS_BILINEAR, nullptr, nullptr, nullptr);
    
    if (!sws_context_) {
        std::cerr << "Could not create SWS context" << std::endl;
        return false;
    }
    
    // Allocate frame buffer for RGB input (if needed for resizing)
    // For now, we'll use the input directly, but this buffer can be used
    // if we need to handle different input sizes
    frame_buffer_ = nullptr; // Will be allocated if we need to resize
    
    // Write header
    ret = avformat_write_header(format_context_, nullptr);
    if (ret < 0) {
        std::cerr << "Error writing header: " << ret << std::endl;
        return false;
    }
    
    return true;
}

bool VideoRecorder::addFrame(const uint8_t* rgb_data, int input_width, int input_height)
{
    if (!is_recording_ || !rgb_data) {
        return false;
    }
    
    std::lock_guard<std::mutex> lock(recording_mutex_);
    
    // If input dimensions not provided, assume they match recording dimensions
    if (input_width == 0) input_width = width_;
    if (input_height == 0) input_height = height_;
    
    if (!encodeFrame(rgb_data, input_width, input_height)) {
        return false;
    }
    
    frame_count_++;
    return true;
}

bool VideoRecorder::encodeFrame(const uint8_t* rgb_data, int input_width, int input_height)
{
    if (!codec_context_ || !frame_) {
        return false;
    }
    
    int ret;
    
    // Make sure frame is writable
    ret = av_frame_make_writable(frame_);
    if (ret < 0) {
        return false;
    }
    
    // Convert RGB to YUV420P
    if (rgb_data && sws_context_) {
        // If input dimensions differ from recording dimensions, we need to update SWS context
        // or crop the input. For now, we'll crop if input is larger, or use the input as-is if smaller
        int src_width = (input_width > width_) ? width_ : input_width;
        int src_height = (input_height > height_) ? height_ : input_height;
        
        // If dimensions changed, we need to recreate the SWS context
        static int last_input_width = 0;
        static int last_input_height = 0;
        if (last_input_width != input_width || last_input_height != input_height) {
            if (sws_context_) {
                sws_freeContext(sws_context_);
            }
            sws_context_ = sws_getContext(
                src_width, src_height, AV_PIX_FMT_RGB24,
                width_, height_, AV_PIX_FMT_YUV420P,
                SWS_BILINEAR, nullptr, nullptr, nullptr);
            if (!sws_context_) {
                std::cerr << "Could not recreate SWS context" << std::endl;
                return false;
            }
            last_input_width = input_width;
            last_input_height = input_height;
        }
        
        uint8_t* src_data[1] = {(uint8_t*)rgb_data};
        int src_linesize[1] = {input_width * 3}; // RGB stride of input
        
        // Scale/crop to the exact codec dimensions (which are even)
        sws_scale(sws_context_,
                  src_data, src_linesize, 0, src_height,
                  frame_->data, frame_->linesize);
        
        // Set frame timestamp
        frame_->pts = frame_count_;
    }
    
    // Send frame to encoder
    ret = avcodec_send_frame(codec_context_, rgb_data ? frame_ : nullptr);
    if (ret < 0) {
        std::cerr << "Error sending frame: " << ret << std::endl;
        return false;
    }
    
    // Receive encoded packets
    while (ret >= 0) {
        ret = avcodec_receive_packet(codec_context_, packet_);
        if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
            break;
        } else if (ret < 0) {
            std::cerr << "Error encoding frame: " << ret << std::endl;
            return false;
        }
        
        // Rescale packet timestamp
        av_packet_rescale_ts(packet_, codec_context_->time_base, video_stream_->time_base);
        packet_->stream_index = video_stream_->index;
        
        // Write packet
        ret = av_interleaved_write_frame(format_context_, packet_);
        if (ret < 0) {
            std::cerr << "Error writing frame: " << ret << std::endl;
            return false;
        }
        
        av_packet_unref(packet_);
    }
    
    return true;
}

void VideoRecorder::cleanup()
{
    if (sws_context_) {
        sws_freeContext(sws_context_);
        sws_context_ = nullptr;
    }
    
    if (frame_buffer_) {
        av_free(frame_buffer_);
        frame_buffer_ = nullptr;
    }
    
    if (frame_) {
        av_frame_free(&frame_);
    }
    
    if (packet_) {
        av_packet_free(&packet_);
    }
    
    if (codec_context_) {
        avcodec_free_context(&codec_context_);
    }
    
    if (format_context_ && format_context_->pb) {
        avio_closep(&format_context_->pb);
    }
    
    if (format_context_) {
        avformat_free_context(format_context_);
        format_context_ = nullptr;
    }
    
    video_stream_ = nullptr;
}


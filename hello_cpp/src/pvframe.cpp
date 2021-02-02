#include <iostream>
#include <sstream>
#include <mutex>
#include <cstring>
#include <memory>
#include <iomanip>
#include <openssl/md5.h>


using namespace std;

//std::string hexdigest(std::string const &s) {
//    return s;
//
//}
//std::string hexdigest(void* const data, size_t len) {
//    return std::string{};
//}


std::string hexdigest(void* const data, size_t len) {
//    return " ???";
    MD5_CTX ctx;
    MD5_Init(&ctx);
    MD5_Update(&ctx, data, len);
    std::string hexdig {2*MD5_DIGEST_LENGTH + 1, 0};
    std::stringstream ss("");
    auto rawdigest = new unsigned char[MD5_DIGEST_LENGTH];
    auto buf = new char[8];
    memset(rawdigest, 0, MD5_DIGEST_LENGTH);
    MD5_Final(rawdigest, &ctx);
    for(unsigned i=0; i <MD5_DIGEST_LENGTH; i++) {
        ss << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(rawdigest[i]);
    }
    delete[] rawdigest;
    delete[] buf;
    std::string const out = ss.str();
    return out;
}



std::string hexdigest(std::string const &s) {
    return hexdigest( (void*)&s[0], s.size());
}

typedef enum
{
    ePvBayerRGGB        = 0,            // First line RGRG, second line GBGB...
    ePvBayerGBRG        = 1,            // First line GBGB, second line RGRG...
    ePvBayerGRBG        = 2,            // First line GRGR, second line BGBG...
    ePvBayerBGGR        = 3,            // First line BGBG, second line GRGR...
    __ePvBayer_force_32 = 0xFFFFFFFF

} tPvBayerPattern;


typedef enum
{
    ePvErrSuccess       = 0,        // No error
    ePvErrCameraFault   = 1,        // Unexpected camera fault

} tPvErr;

typedef enum
{
    ePvFmtMono8         = 0,            // Monochrome, 8 bits
    ePvFmtMono16        = 1,            // Monochrome, 16 bits, data is LSB aligned
    ePvFmtBayer8        = 2,            // Bayer-color, 8 bits

} tPvImageFormat;

typedef struct {
    // --- In
    void*               ImageBuffer;        // Buffer for image/pixel data.
    unsigned long       ImageBufferSize;    // Size of ImageBuffer in bytes
    void*               Context[4];         // For your use.
    unsigned long       _reserved1[8];

    //----- Out -----

    tPvErr              Status;             // Status of this frame
    unsigned long       ImageSize;          // Image size, in bytes
    unsigned long       Width;              // Image width
    unsigned long       Height;             // Image height
    tPvImageFormat      Format;             // Image format
    unsigned long       FrameCount;         // Frame counter. Uses 16bit GigEVision BlockID. Rolls at 65535.
} tPvFrameMini;

typedef struct
{
    //----- In -----
    void*               ImageBuffer;        // Buffer for image/pixel data.
    unsigned long       ImageBufferSize;    // Size of ImageBuffer in bytes

    void*               AncillaryBuffer;    // Camera Firmware >= 1.42 Only.
    unsigned long       AncillaryBufferSize;// Size of your ancillary buffer in bytes. See NonImagePayloadSize attr.
    //   Set to 0 for no buffer.

    void*               Context[4];         // For your use. Possible application: unique ID
    //   of tPvFrame for frame-done callback.
    unsigned long       _reserved1[8];

    //----- Out -----

    tPvErr              Status;             // Status of this frame

    unsigned long       ImageSize;          // Image size, in bytes
    unsigned long       AncillarySize;      // Ancillary data size, in bytes

    unsigned long       Width;              // Image width
    unsigned long       Height;             // Image height
    unsigned long       RegionX;            // Start of readout region (left)
    unsigned long       RegionY;            // Start of readout region (top)
    tPvImageFormat      Format;             // Image format
    unsigned long       BitDepth;           // Number of significant bits
    tPvBayerPattern     BayerPattern;       // Bayer pattern, if bayer format

    unsigned long       FrameCount;         // Frame counter. Uses 16bit GigEVision BlockID. Rolls at 65535.
    unsigned long       TimestampLo;        // Time stamp, lower 32-bits
    unsigned long       TimestampHi;        // Time stamp, upper 32-bits

    unsigned long       _reserved2[32];

} tPvFrame;



tPvFrame *frameNew(unsigned long width, unsigned long height) {
    auto out = new tPvFrame;
    auto size = width * height;
    out->ImageBufferSize = size;
    out->AncillaryBufferSize = 0;
    out->Width  = width;
    out->Height = height;
    out->Format = tPvImageFormat::ePvFmtBayer8;
    out->ImageBuffer = malloc(size);
    if (!out->ImageBuffer) {
        throw std::overflow_error("malloc failed");
    }
    memset(out->ImageBuffer, 0, out->ImageBufferSize);
    return out;
}

void frameFree(tPvFrame *frame) {
    free(frame->ImageBuffer);
    delete frame;
}

class PvFrameWrapper {
public:
    PvFrameWrapper(tPvFrame *frame) {
        frame_.AncillaryBufferSize  = frame->AncillaryBufferSize;
        frame_.ImageBufferSize  = frame->ImageBufferSize;
        frame_.ImageSize        = frame->ImageSize;
        frame_.Status           = frame->Status;
        frame_.Width            = frame->Width;
        frame_.Height           = frame->Height;
        frame_.RegionX          = frame->RegionX;
        frame_.RegionY          = frame->RegionY;
        frame_.Format           = frame->Format;
        frame_.BitDepth         = frame->BitDepth;
        frame_.BayerPattern     = frame->BayerPattern;
        frame_.TimestampLo      = frame->TimestampLo;
        frame_.TimestampHi      = frame->TimestampHi;
        frame_.FrameCount       = frame->FrameCount;
        frame_.ImageBuffer = malloc(frame->ImageBufferSize);
        frame_.AncillaryBuffer = malloc(frame->AncillaryBufferSize);
        if (!frame_.ImageBuffer) { throw std::overflow_error("malloc failed on ImageBuffer");}
        if (!frame_.AncillaryBuffer) { throw std::overflow_error("malloc failed on ImageBuffer");}
        std::memcpy(frame_.ImageBuffer, frame->ImageBuffer, frame->ImageBufferSize);
        std::memcpy(frame_.AncillaryBuffer, frame->AncillaryBuffer, frame->AncillaryBufferSize);
        std::memcpy(frame_.Context, frame->Context, sizeof(frame->Context));
        std::memcpy(frame_._reserved1, frame->_reserved1, sizeof(frame->_reserved1));
        std::memcpy(frame_._reserved2, frame->_reserved2, sizeof(frame->_reserved2));
    }

    static std::shared_ptr<PvFrameWrapper> make_shared(tPvFrame *frame) {
        return std::make_shared<PvFrameWrapper>(frame);
    }


    ~PvFrameWrapper() {
        free(frame_.ImageBuffer);
        free(frame_.AncillaryBuffer);
    }
    tPvFrame frame_;
};




std::string to_string(tPvFrame *t) {
//    std::stringstream ss;
//    ss << "tPvFrame@" << t << "{(" << t->Width << "x"<<t->Height << "), f=" << t->Format << ", "
//        << hexdigest(t->ImageBuffer, t->ImageBufferSize) <<"}" << 0;
//    const std::string s = ss.str();
    auto buf = new char[64];
    memset(buf, 0, 64);
    std::string s{"tPvFrame@"};
    sprintf(buf, "%p", t);
    s.append(buf).append("{(").append(to_string(t->Width)).append("x").append(to_string(t->Height)).append("), f=");
    s.append(std::to_string(t->Format)).append(", ");
    std::string const tmp = hexdigest(t->ImageBuffer, t->ImageBufferSize);
    s.append(tmp).append("}");
    delete[] buf;
    return s;
}

std::string to_string(PvFrameWrapper & t) {
    std::stringstream ss;
    ss << "WPvFrame@" << &t << "{(" << t.frame_.Width << "x"<<t.frame_.Height << "), f=" << t.frame_.Format << ", "
            << hexdigest(t.frame_.ImageBuffer, t.frame_.ImageBufferSize) <<"}";
    return ss.str();
//    tPvFrame *pframe = &t.frame_;
//    return to_string(pframe);
}

void consume(PvFrameWrapper *wrapper) {
    cout << "wrap frame " << to_string(*wrapper) << endl;
}
void consume(std::shared_ptr<PvFrameWrapper> wrapper) {
    cout << "wrap frame " << to_string(*wrapper) << endl;
}

std::shared_ptr<tPvFrame> copyPvFrame(tPvFrame *frame) {
    auto out = std::make_shared<tPvFrame>();
    out->Height = frame->Height;
    out->Width = frame->Width;
    out->Format = frame->Format;
    out->ImageBufferSize = frame->ImageBufferSize;
    return out;
}

int main(int argc, char **argv) {

    tPvFrame *frame = frameNew(640, 480);
//    PvFrameWrapper wrapper(frame);
    auto ptr = PvFrameWrapper::make_shared(frame);

    cout << "empty md5: " << hexdigest("") << endl;
    std::string const tmp = to_string(frame);
    cout << "bare frame " << tmp << endl;
//    cout << "wrap frame " << to_string(*ptr) << endl;
    consume(ptr);


    frameFree(frame);
}
/*
 *    WarnBridge – DAB Packet Mode Decoder Implementation
 *    Part of welle.io fork by Tobias / TogeriX-hub
 *
 *    Architecture derived from:
 *      - welle.io DabAudio (softbit pipeline, deinterleave, EEP deconvolution)
 *      - qt-dab data-processor (packet assembly logic, analysed clean-room)
 *
 *    GPL v2, same as welle.io.
 */

#include "dab-packet.h"
#include "eep-protection.h"
#include <iostream>
#include <chrono>

// Interleave map – identical to DabAudio
static const int16_t interleaveMap[] = {0,8,4,12,2,10,6,14,1,9,5,13,3,11,7,15};

// ---------------------------------------------------------------------------
// CRC-16/CCITT over packed bytes (poly=0x1021, init=0xFFFF)
// Used for DAB packet CRC check (ETSI EN 300 401 §B.1)
// Input: packed byte array, length in bytes (CRC is last 2 bytes)
// ---------------------------------------------------------------------------
static uint16_t crc16_bytes(const uint8_t* data, size_t len)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < len; i++) {
        crc ^= static_cast<uint16_t>(data[i]) << 8;
        for (int b = 0; b < 8; b++) {
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
        }
    }
    return crc ^ 0xFFFF;  // DAB uses inverted CRC per ETSI EN 300 401 §B.1
}

// ---------------------------------------------------------------------------
// Constructor
// ---------------------------------------------------------------------------
DabPacket::DabPacket(
        int16_t            fragmentSize,
        int16_t            bitRate,
        ProtectionSettings protection,
        uint16_t           packetAddress,
        ProgrammeHandlerInterface& handler)
    : handler_(handler)
    , packetAddress_(packetAddress)
    , fragmentSize_(fragmentSize)
    , bitRate_(bitRate)
    , mscBuffer_(64 * 32768)
    , running_(true)
{
    // EEP protection only (Journaline subchannels use EEP per ETSI EN 300 401)
    const bool isEepA = (protection.eepProfile == EEPProtectionProfile::EEP_A);
    protectionHandler_ = std::make_unique<EEPProtection>(
            bitRate, isEepA, static_cast<int>(protection.eepLevel));

    // Output vector: bitRate * 24 bits → bitRate * 3 bytes
    outV_.resize(static_cast<size_t>(bitRate) * 24);

    // Interleaver buffers
    for (int i = 0; i < 16; i++) {
        interleaveData_[i].resize(fragmentSize);
    }

    thread_ = std::thread(&DabPacket::run, this);
}

DabPacket::~DabPacket()
{
    running_ = false;
    dataAvailable_.notify_all();
    if (thread_.joinable()) {
        thread_.join();
    }
}

// ---------------------------------------------------------------------------
// process() – called by MscHandler from the OFDM thread with raw softbits
// ---------------------------------------------------------------------------
int32_t DabPacket::process(const softbit_t* v, int16_t cnt)
{
    while (mscBuffer_.GetRingBufferWriteAvailable() <= cnt) {
        if (!running_) return 0;
        std::this_thread::sleep_for(std::chrono::microseconds(1));
    }
    mscBuffer_.putDataIntoBuffer(v, cnt);
    dataAvailable_.notify_all();
    return cnt;
}

// ---------------------------------------------------------------------------
// run() – background thread: deinterleave → deconvolve → dedisperse → assemble
// Mirrors DabAudio::run() exactly for the first three steps.
// ---------------------------------------------------------------------------
void DabPacket::run()
{
    std::clog << "DabPacket: started packetAddr=0x" << std::hex << packetAddress_ << std::dec << "\n";

    std::vector<softbit_t> data(fragmentSize_);
    std::vector<softbit_t> tempX(fragmentSize_);

    while (running_) {
        std::unique_lock<std::mutex> lock(mutex_);
        while (running_ &&
               mscBuffer_.GetRingBufferReadAvailable() <= fragmentSize_) {
            dataAvailable_.wait(lock);
        }
        if (!running_) break;
        lock.unlock();

        mscBuffer_.getDataFromBuffer(data.data(), fragmentSize_);

        // --- Deinterleave (identical to DabAudio) ---
        for (int16_t i = 0; i < fragmentSize_; i++) {
            tempX[i] = interleaveData_[
                (interleaverIndex_ + interleaveMap[i & 017]) & 017][i];
            interleaveData_[interleaverIndex_][i] = data[i];
        }
        interleaverIndex_ = (interleaverIndex_ + 1) & 0x0F;

        if (countForInterleaver_ <= 15) {
            countForInterleaver_++;
            continue;
        }

        // --- EEP Viterbi deconvolution ---
        protectionHandler_->deconvolve(tempX.data(), fragmentSize_, outV_.data());

        // --- Energy dispersal ---
        energyDispersal_.dedisperse(outV_);

        // --- Packet assembly ---
        handleDecodedFrame(outV_);
    }
}

// ---------------------------------------------------------------------------
// handleDecodedFrame
//
// outV_ contains bitRate*24 decoded bits (uint8_t 0/1).
// A frame can contain multiple DAB packets back to back.
// Packet sizes: 24, 48, 72, 96 bytes → 192, 384, 576, 768 bits.
//
// DAB packet bit layout (ETSI EN 300 401 §5.3.2.1):
//   bits  0.. 1: packet_length  (00=24B 01=48B 10=72B 11=96B)
//   bits  2.. 3: continuity_index
//   bits  4.. 5: first_last     (10=first 00=intermediate 01=last 11=single)
//   bits  6..15: packet_address (10 bits)
//   bit  16:     command_flag
//   bits 17..23: useful_data_length (7 bits)
//   bits 24..N*8-17: useful_data (useful_data_length bytes)
//   last 16 bits: CRC-16
// ---------------------------------------------------------------------------
void DabPacket::handleDecodedFrame(const std::vector<uint8_t>& bits)
{
    static const size_t pkt_sizes_bits[4] = { 192, 384, 576, 768 };
    size_t offset = 0;

    while (offset + 16 < bits.size()) {
        uint32_t pkt_len_code = (bits[offset] << 1) | bits[offset + 1];
        size_t pkt_bits = pkt_sizes_bits[pkt_len_code & 3];

        if (offset + pkt_bits > bits.size()) break;

        handlePacket(bits.data() + offset, pkt_bits);
        offset += pkt_bits;
    }
}

// ---------------------------------------------------------------------------
// handlePacket
//
// Processes one DAB packet (in bit form, 0/1 uint8_t).
// Assembles MSC data groups from first/intermediate/last/single packets.
// On completion calls handler_.onJournalineData().
//
// This logic mirrors qt-dab data-processor::handlePacket() (clean-room).
// ---------------------------------------------------------------------------
void DabPacket::handlePacket(const uint8_t* bits, size_t num_bits)
{
    // Minimum sanity check
    if (num_bits < 24 * 8) return;

    // Convert bit-vector to packed bytes for CRC check
    size_t num_bytes = num_bits / 8;
    std::vector<uint8_t> packed(num_bytes);
    for (size_t i = 0; i < num_bytes; i++) {
        uint8_t b = 0;
        for (int j = 0; j < 8; j++) {
            b = (b << 1) | (bits[i * 8 + j] & 1);
        }
        packed[i] = b;
    }

    // CRC check: CRC-16 over all bytes except last 2, result must equal last 2
    if (num_bytes < 3) return;
    uint16_t crc_calc = crc16_bytes(packed.data(), num_bytes - 2);
    uint16_t crc_pkt  = (static_cast<uint16_t>(packed[num_bytes - 2]) << 8)
                       | packed[num_bytes - 1];
    if (crc_calc != crc_pkt) {
        static int crcErrors = 0;
        if (++crcErrors == 1)
            std::clog << "DabPacket: first CRC error (signal quality)\n";
        return;
    }

    // Parse header from bits
    uint8_t cntIdx    = (bits[2] << 1) | bits[3];
    uint8_t firstLast = (bits[4] << 1) | bits[5];
    uint16_t paddr    = 0;
    for (int i = 0; i < 10; i++) paddr = (paddr << 1) | bits[6 + i];
    uint8_t udlen = 0;
    for (int i = 0; i < 7; i++) udlen = (udlen << 1) | bits[17 + i];

    // Address filter
    if (paddr != packetAddress_) return;

    // Skip if continuity index unchanged and not first packet
    if (cntIdx == static_cast<uint8_t>(lastCntIdx_) && firstLast != 2) return;

    // Continuity check – reset assembler on gap
    if (lastCntIdx_ >= 0 &&
        cntIdx != static_cast<uint8_t>((lastCntIdx_ + 1) % 4)) {
        assembling_ = false;
    }
    lastCntIdx_ = cntIdx;

    // Useful data starts at bit 24, length = udlen bytes = udlen*8 bits
    size_t data_offset_bits = 24;
    size_t data_bits = static_cast<size_t>(udlen) * 8;

    if (data_offset_bits + data_bits + 16 > num_bits) {
        // udlen exceeds packet
        assembling_ = false;
        return;
    }

    switch (firstLast) {
        case 2: { // First packet of data group
            series_.resize(data_bits);
            for (size_t i = 0; i < data_bits; i++)
                series_[i] = bits[data_offset_bits + i];
            assembling_ = true;
            break;
        }
        case 0: { // Intermediate packet
            if (!assembling_) return;
            size_t cur = series_.size();
            series_.resize(cur + data_bits);
            for (size_t i = 0; i < data_bits; i++)
                series_[cur + i] = bits[data_offset_bits + i];
            break;
        }
        case 1: { // Last packet
            if (!assembling_) return;
            size_t cur = series_.size();
            series_.resize(cur + data_bits);
            for (size_t i = 0; i < data_bits; i++)
                series_[cur + i] = bits[data_offset_bits + i];

            // Complete data group assembled – parse Journaline
            dispatchDataGroup(series_);
            series_.resize(0);
            assembling_ = false;
            break;
        }
        case 3: { // Single packet (complete data group)
            series_.resize(data_bits);
            for (size_t i = 0; i < data_bits; i++)
                series_[i] = bits[data_offset_bits + i];
            dispatchDataGroup(series_);
            series_.resize(0);
            assembling_ = false;
            break;
        }
    }
}

// ---------------------------------------------------------------------------
// dispatchDataGroup
//
// series_ contains the complete MSC data group as a bit-vector (0/1 uint8_t).
// Convert to bytes and parse the Journaline data group header +  object.
//
// MSC data group header (ETSI EN 300 401 §5.3.3.1 / dabdatagroupdecoder.h):
//   bit 0:     extension_flag
//   bit 1:     crc_flag
//   bit 2:     segment_flag
//   bit 3:     user_access_flag
//   bits 4..7: data_group_type
//   bits 8..11: continuity_index
//   bits 12..15: repetition_index
//   [optional extension: 16 bits if extension_flag=1]
//   [optional CRC: 16 bits at end if crc_flag=1, over everything before CRC]
//   payload: Journaline NML data
//
// Journaline uses data_group_type=0 ("general data") per dabdatagroupdecoder.h
// ---------------------------------------------------------------------------
void DabPacket::dispatchDataGroup(const std::vector<uint8_t>& bits)
{
    if (bits.size() < 16) return;

    // Convert bit-vector to byte-vector
    size_t num_bytes = bits.size() / 8;
    std::vector<uint8_t> bytes(num_bytes);
    for (size_t i = 0; i < num_bytes; i++) {
        uint8_t b = 0;
        for (int j = 0; j < 8; j++) b = (b << 1) | (bits[i * 8 + j] & 1);
        bytes[i] = b;
    }

    if (bytes.size() < 2) return;

    // Parse MSC data group header
    bool     extension_flag   = (bytes[0] >> 7) & 1;
    bool     crc_flag         = (bytes[0] >> 6) & 1;
    uint8_t  dg_type          = bytes[0] & 0x0F;

    size_t offset = 2;

    // Optional extension field
    if (extension_flag) {
        if (offset + 2 > bytes.size()) return;
        offset += 2;
    }

    // CRC check if present (CRC covers all bytes except the 2-byte CRC itself)
    if (crc_flag) {
        if (bytes.size() < offset + 2) return;
        size_t crc_data_len = bytes.size() - 2;
        uint16_t crc_calc = crc16_bytes(bytes.data(), crc_data_len);
        uint16_t crc_field = (static_cast<uint16_t>(bytes[bytes.size() - 2]) << 8)
                            | bytes[bytes.size() - 1];
        if (crc_calc != crc_field) {
            std::clog << "DabPacket: data group CRC error\n";
            return;
        }
        bytes.resize(crc_data_len);  // strip CRC
    }

    // Journaline uses data_group_type = 0 ("general data")
    if (dg_type != 0) return;

    if (offset >= bytes.size()) return;

    // Remaining bytes are the Journaline NML payload
    // Parse Journaline object header (ETSI TS 102 979 §9):
    //   bytes 0..1: object_id
    //   byte  2:    object_type
    //   byte  3:    number_of_links / title_length marker
    //   ... (type-dependent)
    const uint8_t* payload    = bytes.data() + offset;
    size_t         payload_len = bytes.size() - offset;

    if (payload_len < 4) return;

    uint16_t object_id  = (static_cast<uint16_t>(payload[0]) << 8) | payload[1];
    uint8_t  obj_type   = payload[2];
    // payload[3]: for NewsItem = title_length; for Menu/Root = num_links

    size_t pos = 3;

    std::string title;
    std::string body;

    if (obj_type == 0x02) {
        // NewsItem: byte 3 = title_length, then title bytes, then body bytes
        uint8_t title_len = payload[pos++];
        if (pos + title_len > payload_len) return;
        title = std::string(reinterpret_cast<const char*>(payload + pos), title_len);
        pos += title_len;
        if (pos < payload_len) {
            body = std::string(reinterpret_cast<const char*>(payload + pos),
                               payload_len - pos);
        }
    } else {
        // Menu/Root (0x01, 0x03): byte 3 = num_links, byte 4 = title_length
        pos++; // skip num_links
        if (pos >= payload_len) return;
        uint8_t title_len = payload[pos++];
        if (pos + title_len > payload_len) return;
        title = std::string(reinterpret_cast<const char*>(payload + pos), title_len);
    }

    handler_.onJournalineData(object_id, obj_type, title, body);
}

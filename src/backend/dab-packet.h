/*
 *    WarnBridge – DAB Packet Mode Decoder
 *    Part of welle.io fork by Tobias / TogeriX-hub
 *
 *    Decodes a DAB packet-mode subchannel (e.g. Journaline, DSCTy=0x44a).
 *    Mirrors the structure of DabAudio: softbits → deinterleave →
 *    EEP Viterbi deconvolution → energy dispersal → packet assembly →
 *    MSC data group → ProgrammeHandlerInterface::onJournalineData()
 *
 *    GPL v2, same as welle.io.
 */
#pragma once

#include <cstdint>
#include <vector>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <atomic>
#include <memory>
#include <string>
#include "dab-constants.h"
#include "radio-controller.h"
#include "eep-protection.h"
#include "energy_dispersal.h"
#include "ringbuffer.h"

// Forward declaration
class Protection;

class DabPacket {
public:
    // fragmentSize = subchannel.length * CUSize
    // bitRate      = subchannel.bitrate()
    // protection   = subchannel.protectionSettings
    // packetAddress= ServiceComponent.packetAddress (10-bit DAB packet address)
    // handler      = callback target (onJournalineData will be called)
    DabPacket(
            int16_t              fragmentSize,
            int16_t              bitRate,
            ProtectionSettings   protection,
            uint16_t             packetAddress,
            ProgrammeHandlerInterface& handler);

    ~DabPacket();

    // Called by MscHandler with raw softbits from the CIF
    // (same signature as DabVirtual::process)
    int32_t process(const softbit_t* v, int16_t cnt);

private:
    // Background processing thread (mirrors DabAudio::run)
    void run();

    // Packet assembly (mirrors qt-dab data-processor handlePacket logic)
    void handleDecodedFrame(const std::vector<uint8_t>& bits);
    void handlePacket(const uint8_t* bits, size_t num_bits);
    void dispatchDataGroup(const std::vector<uint8_t>& bits);

    // Extract n bits from bit-vector starting at offset, return as uint32_t
    static uint32_t getBits(const uint8_t* bits, size_t offset, size_t n);

    // CRC-16/CCITT check over bit-vector (len in bits)
    static bool checkCRC(const uint8_t* bits, size_t len_bits);

    ProgrammeHandlerInterface& handler_;
    uint16_t                   packetAddress_;
    int16_t                    fragmentSize_;
    int16_t                    bitRate_;

    std::unique_ptr<EEPProtection> protectionHandler_;
    EnergyDispersal            energyDispersal_;

    RingBuffer<softbit_t>      mscBuffer_;
    std::mutex                 mutex_;
    std::condition_variable    dataAvailable_;
    std::atomic<bool>          running_;
    std::thread                thread_;

    // Interleaver state (identical to DabAudio)
    std::vector<softbit_t>     interleaveData_[16];
    int16_t                    interleaverIndex_   = 0;
    int16_t                    countForInterleaver_= 0;

    // Decoded output buffer (bitRate * 24 bits)
    std::vector<uint8_t>       outV_;

    // Packet assembly state
    bool                       assembling_    = false;
    int16_t                    lastCntIdx_    = -1;
    std::vector<uint8_t>       series_;        // accumulated bits of current datagroup
};

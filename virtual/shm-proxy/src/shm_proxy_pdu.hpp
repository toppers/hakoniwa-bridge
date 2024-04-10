#ifndef _SHM_PROXY_PDU_HPP_
#define _SHM_PROXY_PDU_HPP_

#include "hako_asset_pdu.hpp"
#include "zenoh_comm.hpp"
#include <mutex>
#include <unordered_map>
#include <memory>

struct PduBuffer {
    std::unique_ptr<char[]> data;
    size_t length;
    int lchannel;
    PduBuffer(int ch, size_t len) : data(new char[len]), length(len), lchannel(ch) {}
};
struct PduReader {
    z_owned_publisher_t pub;
    PduBuffer buffer;
    PduReader(int ch, size_t len) : buffer(ch, len) {}
};
struct PduWriter {
    z_owned_subscriber_t sub;
    PduBuffer buffer;
    PduWriter(int ch, size_t len) : buffer(ch, len) {}
};

typedef struct ShmProxyPdu {
    std::vector<hako::asset::Robot> robots;
    ZenohSessionType session;
    std::mutex mutex;
    std::unordered_map<std::string, PduReader> pdu_reader_map;
    std::unordered_map<std::string, PduWriter> pdu_writer_map;
} ShmProxyPduType;

extern bool shm_proxy_pdu_data_initialize(ShmProxyPduType& shm_proxy_pdu);
extern bool shm_proxy_pdu_writer_sync(ShmProxyPduType& shm_proxy_pdu);
extern bool shm_proxy_pdu_reader_sync(ShmProxyPduType& shm_proxy_pdu);

extern ShmProxyPduType* hakoniwa_shm_proxy_pdu;

#endif /* _SHM_PROXY_PDU_HPP_ */
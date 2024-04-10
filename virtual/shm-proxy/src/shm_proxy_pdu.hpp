#ifndef _SHM_PROXY_PDU_HPP_
#define _SHM_PROXY_PDU_HPP_

#include "hako_asset_pdu.hpp"
#include "zenoh_comm.hpp"
#include <atomic>
#include <unordered_map>
#include <memory>

struct PduBuffer {
    std::unique_ptr<char[]> data;
    size_t length;
    int lchannel;
    PduBuffer(int ch, size_t len) : data(new char[len]), length(len), lchannel(ch) {}
};

typedef struct {
    std::vector<hako::asset::Robot> robots;
    ZenohSessionType session;
    std::atomic_bool spin_lock{false};
    std::unordered_map<std::string, PduBuffer> pdu_reader_map;
    std::unordered_map<std::string, PduBuffer> pdu_writer_map;
} ShmProxyPduType;

extern bool shm_proxy_pdu_data_initialize(ShmProxyPduType& shm_proxy_pdu);

#endif /* _SHM_PROXY_PDU_HPP_ */
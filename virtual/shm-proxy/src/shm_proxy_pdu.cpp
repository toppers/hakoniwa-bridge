#include "shm_proxy_pdu.hpp"
#include <iostream>

static void shm_proxy_pdu_data_handler(const z_sample_t *sample, void *arg) {
    z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);
    std::string topic_name(z_loan(keystr));
    std::cout << "Receive: topic_name: " << topic_name << std::endl;
    auto it = hakoniwa_shm_proxy_pdu->pdu_reader_map.find(topic_name);
    if (it != hakoniwa_shm_proxy_pdu->pdu_reader_map.end()) {
        PduReader& reader = it->second;
        if (sample->payload.len > reader.buffer.length) {
            std::cerr << "ERROR: topic data len too large: :" << sample->payload.len << std::endl;
        }
        else {
            while (hakoniwa_shm_proxy_pdu->spin_lock.exchange(true, std::memory_order_acquire)) {}
            memcpy(reader.buffer.data.get(), sample->payload.start, sample->payload.len);
            hakoniwa_shm_proxy_pdu->spin_lock.store(false, std::memory_order_release);
        }
    } else {
        std::cerr << "ERROR: not find topic on map" << std::endl;
    }
    z_drop(z_move(keystr));
}

bool shm_proxy_pdu_data_initialize(ShmProxyPduType& shm_proxy_pdu)
{
    for (const hako::asset::Robot& robot : shm_proxy_pdu.robots) {
        for (const hako::asset::PduWriter& writer : robot.pdu_writers) {
            PduWriter w(writer.channel_id, writer.pdu_size);
            w.pub = z_declare_publisher(z_loan(shm_proxy_pdu.session.s), z_keyexpr(writer.name.c_str()), NULL);
            if (!z_check(w.pub)) {
                std::cout << "Unable to declare Publisher for key expression!" << std::endl;
                return false;
            }
            auto result = shm_proxy_pdu.pdu_writer_map.emplace(writer.name, std::move(w));
            if (!result.second) {
                std::cout << "Failed to insert PduWriter into map." << std::endl;
                return false;
            }
        }
        for (const hako::asset::PduReader& reader : robot.pdu_readers) {
            PduReader r(reader.channel_id, reader.pdu_size);
            z_owned_closure_sample_t callback = z_closure(shm_proxy_pdu_data_handler);
            r.sub = z_declare_subscriber(z_loan(shm_proxy_pdu.session.s), z_keyexpr(reader.name.c_str()), z_move(callback), NULL);
            auto result = shm_proxy_pdu.pdu_reader_map.emplace(reader.name, std::move(r));
            if (!result.second) {
                std::cout << "Failed to insert PduReader into map." << std::endl;
                return false;
            }
        }
    }
    return true;
}
bool shm_proxy_pdu_writer_sync(ShmProxyPduType& shm_proxy_pdu)
{
    //TODO
    return true;
}
bool shm_proxy_pdu_reader_sync(ShmProxyPduType& shm_proxy_pdu)
{
    //TODO
    return true;
}

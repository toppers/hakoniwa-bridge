#include "shm_proxy_pdu.hpp"
#include "hako_asset.h"
#include <iostream>

//#define HAKO_ZENOH_DEBUG_RX
//#define HAKO_ZENOH_DEBUG_TX

static void shm_proxy_pdu_data_handler(const z_sample_t *sample, void *) {
    z_owned_str_t keystr = z_keyexpr_to_string(sample->keyexpr);
    std::string topic_name(z_loan(keystr));
#ifdef HAKO_ZENOH_DEBUG_RX
    std::cout << "Receive: topic_name: " << topic_name << std::endl;
#endif
    auto it = hakoniwa_shm_proxy_pdu->pdu_writer_map.find(topic_name);
    if (it != hakoniwa_shm_proxy_pdu->pdu_writer_map.end()) {
        PduWriter& writer = it->second;
        if (sample->payload.len > writer.buffer.length) {
            std::cerr << "ERROR: topic data len too large: :" << sample->payload.len << std::endl;
        }
        else {
            std::lock_guard<std::mutex> lock(hakoniwa_shm_proxy_pdu->mutex);
            memcpy(writer.buffer.data.get(), sample->payload.start, sample->payload.len);
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
            if (writer.write_cycle == 0) {
                std::cerr << "ERROR: write_cycle is invalid(0). topic_name: " << writer.name << std::endl;
                return false;
            }
            PduWriter w(writer.channel_id, writer.pdu_size, writer.write_cycle);

            z_owned_closure_sample_t callback = z_closure(shm_proxy_pdu_data_handler);
            std::cout << "INFO: Zenoh subscriber: " << writer.name << std::endl;
            w.sub = z_declare_subscriber(z_loan(shm_proxy_pdu.session.s), z_keyexpr(writer.name.c_str()), z_move(callback), NULL);
            auto result = shm_proxy_pdu.pdu_writer_map.emplace(writer.name, std::move(w));
            if (!result.second) {
                std::cout << "Failed to insert PduWriter into map." << std::endl;
                return false;
            }
            std::cout << "INFO: subscriber topic name = " << writer.name << std::endl;
        }
        for (const hako::asset::PduReader& reader : robot.pdu_readers) {
            if (reader.write_cycle == 0) {
                std::cerr << "ERROR: write_cycle is invalid(0). topic_name: " << reader.name << std::endl;
                return false;
            }
            PduReader r(reader.channel_id, reader.pdu_size, reader.write_cycle);

            std::cout << "INFO: Zenoh publisher: " << reader.name << std::endl;
            r.pub = z_declare_publisher(z_loan(shm_proxy_pdu.session.s), z_keyexpr(reader.name.c_str()), NULL);
            if (!z_check(r.pub)) {
                std::cerr << "Unable to declare Publisher for key expression!" << std::endl;
                return false;
            }
            auto result = shm_proxy_pdu.pdu_reader_map.emplace(reader.name, std::move(r));
            if (!result.second) {
                std::cerr << "Failed to insert PduWriter into map." << std::endl;
                return false;
            }
            std::cout << "INFO: publisher topic name = " << reader.name << std::endl;
        }
    }
    return true;
}
bool shm_proxy_pdu_reader_sync(ShmProxyPduType& shm_proxy_pdu)
{
    for (const hako::asset::Robot& robot : shm_proxy_pdu.robots) {
        for (const hako::asset::PduReader& reader : robot.pdu_readers) {
            auto it = hakoniwa_shm_proxy_pdu->pdu_reader_map.find(reader.name.c_str());
            if (it != hakoniwa_shm_proxy_pdu->pdu_reader_map.end()) {
                PduReader& pdu_reader = it->second;
                pdu_reader.buffer.cycle_count++;
                if (pdu_reader.buffer.cycle_count < pdu_reader.buffer.cycle) {
                    continue;
                }
                pdu_reader.buffer.cycle_count = 0;
#ifdef HAKO_ZENOH_DEBUG_TX
                std::cout << "INFO: read pdu: robot = " << robot.name << " channel_id = " << reader.channel_id << " pdu_size = " << reader.pdu_size << std::endl;
#endif
                int ret = hako_asset_pdu_read(robot.name.c_str(), reader.channel_id, (char*)(pdu_reader.buffer.data.get()), reader.pdu_size);
                if (ret != 0) {
                    std::cerr << "ERROR: can not read pdu: robot = " << robot.name << " channel_id = " << reader.channel_id << " pdu_size = " << reader.pdu_size << std::endl;
                    return false;
                }
                else {
                    //std::cout << "publish" << std::endl;
                    z_publisher_put_options_t options = z_publisher_put_options_default();
                    int ret = z_publisher_put(z_loan(pdu_reader.pub), (const uint8_t *)pdu_reader.buffer.data.get(), reader.pdu_size, &options);
                    if (ret != 0) {
                        std::cerr << "ERROR: can not publish: robot = " << robot.name << " channel_id = " << reader.channel_id << " pdu_size = " << reader.pdu_size << std::endl;
                    }
                    else {
                        //nothing to do
                    }
                }
            } else {
                std::cerr << "ERROR: can not find pdu: robot = " << robot.name << " channel_id = " << reader.channel_id << " pdu_size = " << reader.pdu_size << std::endl;
                return false;
            }
        }
    }
    return true;
}

bool shm_proxy_pdu_writer_sync(ShmProxyPduType& shm_proxy_pdu)
{
    for (const hako::asset::Robot& robot : shm_proxy_pdu.robots) {
        for (const hako::asset::PduWriter& writer : robot.pdu_writers) {
            auto it = hakoniwa_shm_proxy_pdu->pdu_writer_map.find(writer.name.c_str());
            if (it != hakoniwa_shm_proxy_pdu->pdu_writer_map.end()) {
                PduWriter& pdu_writer = it->second;
                pdu_writer.buffer.cycle_count++;
                if (pdu_writer.buffer.cycle_count < pdu_writer.buffer.cycle) {
                    continue;
                }
                pdu_writer.buffer.cycle_count = 0;
                std::lock_guard<std::mutex> lock(hakoniwa_shm_proxy_pdu->mutex);
                int ret = hako_asset_pdu_write(robot.name.c_str(), writer.channel_id, (const char*)(pdu_writer.buffer.data.get()), writer.pdu_size);
                if (ret != 0) {
                    std::cerr << "ERROR: can not write pdu: robot = " << robot.name << " channel_id = " << writer.channel_id << " pdu_size = " << writer.pdu_size << std::endl;
                    return false;
                }
                else {
                    //nothing to do
                }
            } else {
                std::cerr << "ERROR: can not find pdu: robot = " << robot.name << " channel_id = " << writer.channel_id << " pdu_size = " << writer.pdu_size << std::endl;
                return false;
            }
        }
    }
    return true;
}

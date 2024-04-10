#include "shm_proxy_pdu.hpp"

bool shm_proxy_pdu_data_initialize(ShmProxyPduType& shm_proxy_pdu)
{
    for (const hako::asset::Robot& robot : shm_proxy_pdu.robots) {
        for (const hako::asset::PduWriter& writer : robot.pdu_writers) {
            shm_proxy_pdu.pdu_writer_map.emplace(writer.name, PduBuffer(writer.channel_id, writer.pdu_size));
        }
        for (const hako::asset::PduReader& reader : robot.pdu_readers) {
            shm_proxy_pdu.pdu_reader_map.emplace(reader.name, PduBuffer(reader.channel_id, reader.pdu_size));
        }
    }
    return true;
}
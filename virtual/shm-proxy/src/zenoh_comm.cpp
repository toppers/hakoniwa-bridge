#include "zenoh_comm.hpp"
#include <iostream>

bool zenoh_initialize(ZenohSessionType& session)
{
    z_owned_config_t config = z_config_default();
    std::cout << "INFO: Zenoh Opening session..." << std::endl;
    session.s = z_open(z_move(config));
    if (!z_check(session.s)) {
        std::cerr << "ERROR: Zenoh Unable to open session!" << std::endl;
        return false;
    }
    return true;
}

void zenoh_finalize(ZenohSessionType& session)
{
    z_close(z_move(session.s));
}

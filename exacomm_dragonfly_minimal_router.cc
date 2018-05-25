#include <sstmac/hardware/topology/structured_topology.h>
#include <sstmac/hardware/switch/network_switch.h>
#include <sprockit/util.h>
#include <sprockit/sim_parameters.h>

#include "exacomm_dragonfly_minimal_router.h"  


namespace sstmac {
namespace hw {

exacomm_dragonfly_minimal_router::exacomm_dragonfly_minimal_router(sprockit::sim_parameters *params, topology *top, network_switch *netsw)
  :  router(params, top, netsw)
{

  sid_ = addr();
  dtop_ = safe_cast(exacomm_dragonfly_topology, top);
};

void exacomm_dragonfly_minimal_router::route(packet* pack) {
  packet::path& path = pack->current_path();
  switch_id my_addr = my_addr_;
  uint16_t port;
  switch_id ej_addr = dtop_->node_to_injection_switch(pack->toaddr(), port);
  if (my_addr_ == ej_addr) {
  	path.set_outport(port);
    return;
  } else {
  	dtop_->minimal_route_to_switch(my_addr, ej_addr, path);
  }
  return;
};

}
}
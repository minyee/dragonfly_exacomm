
#include <sstmac/hardware/topology/structured_topology.h>
#include <sstmac/hardware/switch/network_switch.h>
#include <sprockit/util.h>
#include <sprockit/sim_parameters.h>
#include <sstmac/common/event_manager.h>

#include "exacomm_dragonfly_par_router.h"  



namespace sstmac {
namespace hw {



exacomm_dragonfly_par_router::exacomm_dragonfly_par_router(sprockit::sim_parameters *params, topology *top, network_switch *netsw)
  :  ugal_router(params, top, netsw)
{
  val_threshold_ = params->get_optional_int_param("ugal_threshold", 0);
  val_preference_factor_ = params->get_optional_int_param("valiant_preference_factor",1);
  dtop_ = safe_cast(exacomm_dragonfly_topology, top);
  std::cout << "EXACOMM DRAGONFLY PAR ROUTER" << std::endl;
};


bool exacomm_dragonfly_par_router::switch_paths(switch_id orig_dst, 
                                        switch_id new_dst, 
                                        packet::path& orig_path, 
                                        packet::path& new_path) {
  switch_id src = my_addr_;
  dtop_->minimal_route_to_switch(src, orig_dst, orig_path);
  dtop_->minimal_route_to_switch(src, new_dst, new_path);
  int orig_queue_length = netsw_->queue_length(orig_path.outport());
  int new_queue_length = netsw_->queue_length(new_path.outport());
  int orig_distance = dtop_->minimal_distance(src, orig_dst);
  int new_distance = dtop_->minimal_distance(src, new_dst)
                      + dtop_->minimal_distance(new_dst, orig_dst);
  int orig_weight = orig_queue_length * orig_distance * val_preference_factor_;
  int valiant_weight = new_queue_length * new_distance;
  return valiant_weight < orig_weight;
}


void exacomm_dragonfly_par_router::route(packet* pkt) {
  // make sure that the interconnect pointer isnt null
  uint16_t out_port;
  uint16_t in_port;
  switch_id ej_addr = dtop_->node_to_ejection_switch(pkt->toaddr(), out_port);
  switch_id inj_addr = dtop_->node_to_ejection_switch(pkt->fromaddr(), in_port);
  auto hdr = pkt->get_header<header>();
  packet::path& pth = pkt->current_path();
  // when this router is the injection switch iteself
  if (inj_addr == my_addr_) {
    hdr->stage = initial_stage;
  }
  // if we are at the ejection switch
  if (ej_addr == my_addr_) {
    pth.set_outport(out_port);
    return;
  }

  switch(hdr->stage) {
    case(initial_stage): {
      packet::path original_path;
      packet::path valiant_path;
      switch_id intermediate_switch = dtop_->random_intermediate_switch(my_addr_, ej_addr, seed_);
      if (switch_paths(ej_addr, intermediate_switch, original_path, valiant_path)) {
        hdr->stage = valiant_stage;
        pkt->set_dest_switch(intermediate_switch);
        pkt->current_path() = valiant_path;
      } else {
        pkt->set_dest_switch(ej_addr);
        pkt->current_path() = original_path;  
      }
      break;
    }
    case(valiant_stage):
      if (my_addr_ == pkt->dest_switch()) {
        pkt->set_dest_switch(ej_addr);
        hdr->stage = final_stage;
      }
    case(final_stage):
      break;
  }

  packet::path& path = pkt->current_path();
  dtop_->minimal_route_to_switch(my_addr_, pkt->dest_switch(), path);
  return;
};



}
}
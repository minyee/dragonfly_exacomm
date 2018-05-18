
//#include <sstmac/hardware/router/ugal_routing.h>
#include <sstmac/hardware/topology/structured_topology.h>
#include <sstmac/hardware/switch/network_switch.h>
#include <sprockit/util.h>
#include <sprockit/sim_parameters.h>
#include <sstmac/common/event_manager.h>

#include "flexfly_par_router.h"  
#include "flexfly_topology_simplified.h"



namespace sstmac {
namespace hw {



flexfly_par_router::flexfly_par_router(sprockit::sim_parameters *params, topology *top, network_switch *netsw)
  :  ugal_router(params, top, netsw)
{
  std::cout << "par router ayyyyyy" << std::endl;
  val_threshold_ = params->get_optional_int_param("ugal_threshold", 0);
  val_preference_factor_ = params->get_optional_int_param("valiant_preference_factor",1);
  seed_ = params->get_optional_int_param("seed", 30);
  ic_ = nullptr;
  ftop_ = safe_cast(flexfly_topology_simplified, top);
};


void flexfly_par_router::route_initial(packet* pkt, switch_id ej_addr) {
  packet::path& pth = pkt->current_path();
  packet::path min_path;
  packet::path valiant_path;
  switch_id intermediate_group = ftop_->random_intermediate_switch(my_addr_, ej_addr, seed_);
  bool use_alternative_path = switch_paths(ej_addr, 
                                            intermediate_group, 
                                            min_path, 
                                            valiant_path);
  header* hdr = pkt->get_header<header>();  

  if (use_alternative_path) {
    hdr->stage = valiant_stage;
    pkt->set_dest_switch(intermediate_group);
  } else {
    hdr->stage = minimal_stage;
    pkt->set_dest_switch(ej_addr);
  }
};

bool flexfly_par_router::switch_paths(switch_id orig_dst, 
                                        switch_id new_dst, 
                                        packet::path& orig_path, 
                                        packet::path& new_path) {
  switch_id src = my_addr_;
  ftop_->minimal_route_to_switch(src, orig_dst, orig_path);
  ftop_->minimal_route_to_switch(src, new_dst, new_path);
  int orig_queue_length = netsw_->queue_length(orig_path.outport());
  int new_queue_length = netsw_->queue_length(new_path.outport());
  int orig_distance = ftop_->minimal_distance(src, orig_dst);
  int new_distance = ftop_->minimal_distance(src, new_dst)
                      + ftop_->minimal_distance(new_dst, orig_dst);
  int orig_weight = orig_queue_length * orig_distance * val_preference_factor_;
  int valiant_weight = new_queue_length * new_distance;
  return valiant_weight < orig_weight;
}

bool flexfly_par_router::route_common(packet* pkt) {
  uint16_t out_port;
  uint16_t in_port;
  switch_id ej_addr = ftop_->node_to_ejection_switch(pkt->toaddr(), out_port);
  switch_id inj_addr = ftop_->node_to_ejection_switch(pkt->fromaddr(), in_port);
  packet::path& pth = pkt->current_path();
  auto hdr = pkt->get_header<header>();

  uint16_t tmp;
  if (my_addr_ == inj_addr) {
    hdr->stage = initial_stage;
  }

  if (my_addr_ == ej_addr) {
    // checks if we are at the ejection switch yet
    pkt->set_dest_switch(ej_addr);
    pth.set_outport(out_port);
    return true;
  }
  
  switch(hdr->stage) {
    case(initial_stage): 
        // at first we need to decide whether or not there is congestion, if there isn't
        // congestion, set packet to minimal stage, else set it to valiant stage
        route_initial(pkt, ej_addr);
        break;
    case(valiant_stage):
        pkt->set_dest_switch(ej_addr);
        hdr->stage = final_stage;
        break;
    case(minimal_stage):
        pkt->set_dest_switch(ej_addr);
        hdr->stage = final_stage;
        break;
    case(final_stage):
        abort();
        // should never really get here I think
        break;
  }
  
  return false;
};


void flexfly_par_router::route(packet* pkt) {
  // make sure that the interconnect pointer isnt null
  if (ic_ == nullptr) {
    ic_ = netsw_->event_mgr()->interconn();
  }
  bool eject = route_common(pkt);
  if (eject) return; // if eject then no need to route anymore

  packet::path& path = pkt->current_path();
  ftop_->minimal_route_to_switch(my_addr_, pkt->dest_switch(), path);
  return;
};



}
}
#include <sstmac/hardware/topology/structured_topology.h>
#include <sstmac/hardware/switch/network_switch.h>
#include <sprockit/util.h>
#include <sprockit/sim_parameters.h>
#include <sstmac/common/event_manager.h>
#include "exacomm_dragonfly_topology.h"
#include "exacomm_dragonfly_ugalG_router.h"  

namespace sstmac {
namespace hw {

// basically the stage when the packet has just been injected into the 
static const char initial_stage = 0; 

// at the intermediate stage
static const char intermediate_stage = 1; 



struct triple_tuple {
  switch_id intermediate_group;
  int src_outport;
  uint64_t total_queue_length;
  triple_tuple(switch_id id, int outport, uint64_t q_len) : 
        intermediate_group(id), 
        src_outport(outport), 
        total_queue_length(q_len) {};
};

exacomm_dragonfly_ugalG_router::exacomm_dragonfly_ugalG_router(sprockit::sim_parameters *params, topology *top, network_switch *netsw)
  :  ugal_router(params, top, netsw)
{
  
  //val_threshold_ = params->get_optional_int_param("ugal_threshold", 0);
  //val_preference_factor_ = params->get_optional_int_param("valiant_preference_factor",1);
  ic_ = nullptr;
  ftop_ = safe_cast(exacomm_dragonfly_topology_simplified, top);
};



/********************************************************************************************
 **** Main algorithm for finding the best intermediate group to route to
 ********************************************************************************************/
void exacomm_dragonfly_ugalG_router::route_to_intermediate_group_stage(packet* pkt) {
  header* hdr = pkt->get_header<header>();
  std::vector<triple_tuple> intermediate_group_collection;
  switch_id target_group = my_addr_;
  uint16_t port;
  
  switch_id ej_addr = ftop_->node_to_injection_switch(pkt->toaddr(), port);
  
  if (ic_ == nullptr) ic_ = netsw_->event_mgr()->interconn();
  
  //const std::vector<network_switch*>& smap = ic_->switches();
  network_switch* ej_switch = ic_->switch_at(ej_addr);

  std::vector<topology::connection> reachable_groups_from_src;
  
  ftop_->connected_outports(my_addr_, reachable_groups_from_src);
  for (auto global_link : reachable_groups_from_src) {
    
    switch_id intermediate_grp = global_link.dst;
    if (intermediate_grp == my_addr_ || intermediate_grp == ej_addr) continue;
    std::vector<topology::connection> groups_reachable_from_intermediate_group;
    ftop_->connected_outports(intermediate_grp, groups_reachable_from_intermediate_group);
    bool is_connected_to_ej_grp = false;
    network_switch* intermediate_group_switch = ic_->switch_at(intermediate_grp);
    uint64_t intermediate_switch_queue_length = intermediate_group_switch->queue_length(global_link.dst_inport);

    for (auto intermediate_outgoing_link : groups_reachable_from_intermediate_group) {
      if (intermediate_outgoing_link.dst == ej_addr) {

        intermediate_group_collection.push_back(triple_tuple(intermediate_grp, 
                                                global_link.src_outport, 
                                                intermediate_switch_queue_length + ej_switch->queue_length(intermediate_outgoing_link.dst_inport)));
        is_connected_to_ej_grp = true;
        break;
      }
    }
  }
  uint32_t min_queue_len =1e6; 
  for (auto intermediate_grp_tuple : intermediate_group_collection) {
    if (intermediate_grp_tuple.total_queue_length < min_queue_len) {
      target_group = intermediate_grp_tuple.intermediate_group;
      min_queue_len = intermediate_grp_tuple.total_queue_length;
    }
  }
  hdr->stage = intermediate_stage;
  assert(target_group != my_addr_); // not moving any closer to the target
  pkt->set_dest_switch(target_group);
  
  return;
}; 


void exacomm_dragonfly_ugalG_router::route_to_dest(packet* pkt) {
  uint16_t outport;
  switch_id ej_addr = ftop_->node_to_ejection_switch(pkt->toaddr(), outport);
  pkt->set_dest_switch(ej_addr);
}


/********************************************************************************************
 **** Main algorithm for finding the best intermediate group to route to
 ********************************************************************************************/




void exacomm_dragonfly_ugalG_router::route(packet* pkt) {
  uint16_t in_port;
  uint16_t out_port;
  switch_id ej_addr = ftop_->node_to_injection_switch(pkt->toaddr(), out_port);
  switch_id inj_addr = ftop_->node_to_injection_switch(pkt->fromaddr(), in_port);

  if (my_addr_ == inj_addr) {
    header* hdr = pkt->get_header<header>();
    hdr->stage = initial_stage;
  } 
  if (my_addr_ == ej_addr) {
    // All we want is to immediately eject the packet at this switch because we
    // are at the ejection switch 
    packet::path& pth = pkt->current_path();
    pkt->set_outport(out_port);
    return;
  }
  header* hdr = pkt->get_header<header>();
  packet::path& pth = pkt->current_path();
  switch (hdr->stage) {
    case (initial_stage):
      route_to_intermediate_group_stage(pkt);
      //pkt->set_dest_switch(ej_addr);
      break;
    case (intermediate_stage):
      route_to_dest(pkt);
      break;
  }
  ftop_->minimal_route_to_switch(my_addr_, pkt->dest_switch(), pth); // route minimally to said group
  return;
};




}
}
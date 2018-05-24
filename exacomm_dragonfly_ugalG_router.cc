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
  ic_ = nullptr; // HEREEEEEE
  dfly_ = safe_cast(exacomm_dragonfly_topology, top);
};



void exacomm_dragonfly_ugalG_router::find_min_group_link(int grp, int dst_group, switch_id& min_exit_swid, int& min_total_length) {
  // first find the list of all the switches in grp that leads to the final destination group
  int switches_per_group = dfly_->switches_per_group();
  std::vector<std::pair<switch_id, int>> spp; // short for switch_port_pair
  dfly_->switches_connecting_groups(grp, dst_group, spp);
  std::cout << "in find min group link" << std::endl;
  min_total_length = INT_MAX;
  if (!ic_) ic_ = netsw_->event_mgr()->interconn();
  for (auto pair : spp) {
    switch_id entry_point_swid = pair.first;
    std::cout << "entry_point_swid : " << std::to_string(entry_point_swid) << std::endl; 
    network_switch* netsw = ic_->switch_at(entry_point_swid);
    std::cout << "loop2" << std::endl; 
    if (min_total_length > netsw->queue_length(pair.second)) {
      min_total_length = netsw->queue_length(pair.second);
      min_exit_swid = entry_point_swid;
    }
  }
  return;
}

void exacomm_dragonfly_ugalG_router::select_ugalG_intermediate(packet* pkt, switch_id ej_addr) {  
  // we want to select an intermediate group switch, but what if that switch is not a gateway switch at the intermediate group
  std::cout << "select_ugalG_intermediate in " << std::endl;
  // Phase I:
  // find out all the basic information here first, initialize all the relevant information here
  node_id src_nid = pkt->fromaddr();
  switch_id src_swid = dfly_->node_to_switch(src_nid);
  switch_id dst_swid = ej_addr;
  int src_group = dfly_->group_from_swid(src_swid);
  int dst_group = dfly_->group_from_swid(dst_swid);
  int curr_group = dfly_->group_from_swid(my_addr_);
  int min_total_length = 1e8;

  // Phase II:
  if (curr_group == dst_group || src_group == dst_group) return; // do nothing
  std::cout << "000000" << std::endl;
  // if we are here, then we should randomly select an intermediate group
  switch_id final_entry_switch;
  switch_id final_exit_switch;
  
  find_min_group_link(curr_group, dst_group, final_entry_switch, min_total_length);

  for (int grp  = 0; grp < dfly_->num_groups(); grp++) {
    if (grp == curr_group || grp == src_group || grp == dst_group) continue;
    switch_id entry_switch, exit_switch;
    int entry_queue_length, exit_queue_length;
    find_min_group_link(curr_group, grp, entry_switch, entry_queue_length);
    find_min_group_link(grp, dst_group, exit_switch, exit_queue_length);
    if (min_total_length > entry_queue_length + exit_queue_length) {
      min_total_length = entry_queue_length + exit_queue_length;
      final_entry_switch = entry_switch;
      final_exit_switch = exit_switch;
    }
  }
  header* hdr = pkt->get_header<header>();
  hdr->entry_swid = final_entry_switch;
  hdr->exit_swid = final_exit_switch;
  hdr->intermediate_grp = dfly_->group_from_swid(final_exit_switch);
  return;
}


/********************************************************************************************
 ********************************************************************************************
 **** Main algorithm for finding the best intermediate group to route to
 ******************************************************************************************** 
 ********************************************************************************************/

void exacomm_dragonfly_ugalG_router::route(packet* pkt) {
  std::cout << "got in " << std::endl;
  std::cout << "pkt: " << std::endl;
  uint16_t in_port;
  uint16_t out_port;
  switch_id ej_addr = dfly_->node_to_injection_switch(pkt->toaddr(), out_port);
  switch_id inj_addr = dfly_->node_to_injection_switch(pkt->fromaddr(), in_port);
  int dst_group = dfly_->group_from_swid(ej_addr);
  int src_group = dfly_->group_from_swid(inj_addr);
  int curr_group = dfly_->group_from_swid(my_addr_);

  header* hdr = pkt->get_header<header>();
  packet::path& pth = pkt->current_path();

  std::cout << "here1 " << std::endl;
  
  if (my_addr_ == inj_addr) {
    // be sure to initialize everything when we are at the initial stage
    hdr->stage_number = initial_stage; // set the stage to initial stage
    hdr->num_group_hops = 0;
  } 

  if (curr_group == dst_group) {
    // we already are in the destination group, so no need to hop out of this group anymore
    std::cout << "here2 " << std::endl;
    hdr->stage_number = intra_grp_stage;
    if (my_addr_ == ej_addr) {
      // even better, if we've already reached the final destination, just eject the packet to node
      pth.set_outport(out_port);
      std::cout << "got out " << std::endl;
      return;
    }
  }

  if (my_addr_ == ej_addr) {
    // All we want is to immediately eject the packet at this switch because we are at the ejection switch 
    std::cout << "here3 " << std::endl;
    pth.set_outport(out_port);
    std::cout << "got out " << std::endl;
    return;
  }


  std::cout << "here4" << std::endl;

  switch (hdr->stage_number) {
    case (initial_stage):
      select_ugalG_intermediate(pkt, ej_addr);
      hdr->stage_number = hop_to_entry_stage;
      break;
   
    case (hop_to_entry_stage):
      hdr->stage_number = hop_to_exit_stage;
      pkt->set_dest_switch(hdr->entry_swid);
      break;
   
    case (hop_to_exit_stage):
      pkt->set_dest_switch(hdr->exit_swid); // set the destination switch to be the exit switch id which is embedded within the header
      hdr->stage_number = intra_grp_stage;
      break;
   
    case (intra_grp_stage):
      
    case (final_stage):
      dfly_->minimal_route_to_switch(my_addr_, ej_addr, pth); // route minimally to said group
      break;
  }
  




  if (dfly_->is_global_port(my_addr_, pth.outport())) {
    hdr->num_group_hops++;
  }


  assert(hdr->num_group_hops <= 2);
  std::cout << "got out " << std::endl;
  return;
};




}
}
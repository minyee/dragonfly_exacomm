/**
 * Author: Min Yee Teh
 */
#include <vector>
#include <unordered_map>
#include <sstmac/common/node_address.h>
#include <sprockit/sim_parameters.h>
#include <stdlib.h>
#include <queue>
#include <sstmac/hardware/topology/topology.h>
#include "exacomm_dragonfly_topology.h"





namespace sstmac {
namespace hw {
RegisterKeywords(
                  {"groups", "The number of groups"},
                  {"switches_per_group", ""},
                  {"nodes_per_switch", ""},
                  {"adjacency_matrix_filename", ""},
);
 
 exacomm_dragonfly_topology::exacomm_dragonfly_topology(sprockit::sim_parameters* params) : 
                              structured_topology(params,InitMaxPortsIntra::I_Remembered, 
                                                  InitGeomEjectID::I_Remembered) {
  num_groups_ = params->get_int_param("groups"); // controls g
 	switches_per_group_ = params->get_int_param("switches_per_group"); // controls a
 	nodes_per_switch_ = params->get_optional_int_param("nodes_per_switch", 4);

  bool is_canonical = params->get_optional_bool_param("canonical", false); // assume that by default, the dragonfly is not canonical
  bool load_balancing_routing_ = params->get_optional_bool_param("load_balance_routing", false); 
  std::string filename = params->get_param("adjacency_matrix_filename");
  max_switch_id_ = (num_groups_ * switches_per_group_) - 1;

  outgoing_adjacency_list_.resize(num_groups_ * switches_per_group_);
  //incoming_adjacency_list_.resize(num_groups_ * switches_per_group_); // incoming adjacency list is somewhat redundant information
  distance_matrix_.resize(max_switch_id_ + 1);
  switch_usage_.resize(max_switch_id_ + 1);
  routing_table_.resize(max_switch_id_ + 1);
  for (int i = 0; i <= max_switch_id_; i++) {
    distance_matrix_[i].resize(max_switch_id_ + 1);
    routing_table_[i].resize(max_switch_id_ + 1);
  }
  // now figure out what the adjacency matrix of the entire topology looks like
  // more importantly how do I transfer that information from
  if (!is_canonical)
    form_topology(filename);
  else {
    switches_per_group_ = num_groups_ - 1;
    form_canonical_dragonfly();
  }
  diameter_ = 0;
  route_minimal_topology(diameter_);
 };

 exacomm_dragonfly_topology::~exacomm_dragonfly_topology() {
  // remember to go around and free the dfly links
  for (auto dfly_link_vector : outgoing_adjacency_list_) {
    for (dfly_link* dlink : dfly_link_vector) {
      if (dlink) {
        delete dlink;
      }
    }
  }
 };

 /**
  * IMPORTANT: This function will route the minimal path
  **/
 void exacomm_dragonfly_topology::minimal_route_to_switch(switch_id src_switch_addr, 
 												switch_id dst_switch_addr, 
 												routable::path& path) const {
  int dist = 0;
  switch_id curr_switch = dst_switch_addr;
  switch_id parent = curr_switch;
  while (curr_switch != src_switch_addr) {
    if (dist > diameter_)
        spkt_abort_printf("ROUTING IS WRONG!!!!!!!!!!");
    parent = curr_switch;
    curr_switch = routing_table_[src_switch_addr][curr_switch];
    dist++;
  }
    // at this point the parent switch should be the next switch
  for (auto link : outgoing_adjacency_list_[src_switch_addr]) {
    if (link->get_dst() == parent) {
      path.set_outport(link->get_src_outport());
    }
  }
 };


void exacomm_dragonfly_topology::connected_outports(const switch_id src, 
                                            std::vector<topology::connection>& conns) const {
  int size = outgoing_adjacency_list_[src].size();
  conns.resize(size);
  for (int i = 0; i < size; i++) {
    conns[i].src = outgoing_adjacency_list_[src][i]->get_src();
    conns[i].dst = outgoing_adjacency_list_[src][i]->get_dst();
    conns[i].src_outport = outgoing_adjacency_list_[src][i]->get_src_outport(); 
    conns[i].dst_inport = outgoing_adjacency_list_[src][i]->get_dst_inport(); 
  }
}


bool exacomm_dragonfly_topology::switch_id_slot_filled(switch_id sid) const {
  return (sid <= max_switch_id_);
}

void exacomm_dragonfly_topology::configure_vc_routing(std::map<routing::algorithm_t, int>& m) const {
  m.insert({routing::minimal, 3});
  m.insert({routing::minimal_adaptive, 3});
  m.insert({routing::valiant, 3});
  m.insert({routing::ugal, 3});
  return;
};



switch_id exacomm_dragonfly_topology::node_to_ejection_switch(node_id addr, uint16_t& port) const {
  switch_id swid = addr / nodes_per_switch_; // this gives us the switch id of the switch node addr is connected to
  int ind = addr % nodes_per_switch_;
  int offset = outgoing_adjacency_list_[swid].size();
  port = offset + ind;
  return swid;
};
  
  switch_id exacomm_dragonfly_topology::node_to_injection_switch(node_id addr, uint16_t& port) const {
    return node_to_ejection_switch(addr, port);
  };

  /**
   * NOTE: This method does not include the hop to an optical switch
   **/
  int exacomm_dragonfly_topology::minimal_distance(switch_id src, switch_id dst) const {
    return distance_matrix_[src][dst];
  };

  // need to figure out how to implement this function now that we cannot figure out the distance matrix
  // right after initialization
  int exacomm_dragonfly_topology::num_hops_to_node(node_id src, node_id dst) const {
    int src_swid = src / (nodes_per_switch_);
    int dst_swid = dst / (nodes_per_switch_);
    int min_dist = distance_matrix_[src_swid][dst_swid];
    // added by 2 because each node is 1 hop away from it's switch
    return min_dist + 2; 
  };

  void exacomm_dragonfly_topology::nodes_connected_to_injection_switch(switch_id swid, 
                                                              std::vector<injection_port>& nodes) const {
    int port_offset = outgoing_adjacency_list_[swid].size();
    nodes.resize(nodes_per_switch_);
    for (int i = 0; i < nodes_per_switch_; i++) {
      int node_id = swid * nodes_per_switch_ + i;
      int port_ind = port_offset + i;
      //injection_port ijp;
      nodes[i].nid = node_id;
      nodes[i].port = port_ind;
    }
  };

  void exacomm_dragonfly_topology::nodes_connected_to_ejection_switch(switch_id swid, 
                                                              std::vector<injection_port>& nodes) const { 
    nodes_connected_to_injection_switch(swid, nodes);
  };

  // returns the group id of a given switch
  int exacomm_dragonfly_topology::group_from_swid(switch_id swid) const {
    return swid / (switches_per_group_);
  };

  /**
   * prints out the all the connections for each switch
   */
  void exacomm_dragonfly_topology::print_port_connection_for_switch(switch_id swid) const {
    std::cout << "This is connections for switch: " << std::to_string(swid) << std::endl;
    for (auto link : outgoing_adjacency_list_[swid]) {
      std::cout << "Port: " << std::to_string(link->get_src_outport()) << " goes to inport " << std::to_string(link->get_dst_inport())
              << " for switch " << std::to_string(link->get_dst()) << std::endl;
    }
  };


  /**
   * @brief num_endpoints To be distinguished slightly from nodes.
   * Multiple nodes can be grouped together with a netlink.  The netlink
   * is then the network endpoint that injects to the switch topology
   * @return
   */
  int exacomm_dragonfly_topology::num_netlinks() const {
    return 1; // each node is only connected to one endpoint
  }; 

  switch_id exacomm_dragonfly_topology::max_netlink_id() const {
    return 0;
    //return max_switch_id_;
  };

  bool exacomm_dragonfly_topology::netlink_id_slot_filled(node_id nid) const {
    return (nid < (num_groups_ * switches_per_group_ * nodes_per_switch_));
  };

  /**
     For a given node, determine the injection switch
     All messages from this node inject into the network
     through this switch
     @param nodeaddr The node to inject to
     @param switch_port [inout] The port on the switch the node injects on
     @return The switch that injects from the node
  */
  switch_id exacomm_dragonfly_topology::netlink_to_injection_switch(
        netlink_id nodeaddr, uint16_t& switch_port) const {
    return max_switch_id_;
  };

  /**
     For a given node, determine the ejection switch
     All messages to this node eject into the network
     through this switch
     @param nodeaddr The node to eject from
     @param switch_port [inout] The port on the switch the node ejects on
     @return The switch that ejects` into the node
  */
  switch_id exacomm_dragonfly_topology::netlink_to_ejection_switch(
        netlink_id nodeaddr, uint16_t& switch_port) const {
    return max_switch_id_;
  };

  
  bool exacomm_dragonfly_topology::node_to_netlink(node_id nid, node_id& net_id, int& offset) const {
    net_id = nid;
    offset = 0;
    return false;
  };

  switch_id exacomm_dragonfly_topology::node_to_switch(node_id nid) const {
    switch_id swid = nid / (nodes_per_switch_);
    return swid;
  };

  void exacomm_dragonfly_topology::configure_individual_port_params(switch_id src,
          sprockit::sim_parameters* switch_params) const {
    if (!switch_params || src > max_switch_id_) return;
    sprockit::sim_parameters* link_params = switch_params->get_namespace("link");
    double electrical_bw = link_params->get_bandwidth_param("electrical_link_bandwidth");
    double optical_bw = link_params->get_bandwidth_param("optical_link_bandwidth");
    long credits = switch_params->get_optional_long_param("credits", 100000000);
    
    //std::cout << "Switch " << std::to_string(src) << ":" << std::endl;

    for (int i = 0; i < outgoing_adjacency_list_[src].size(); i++) {
      
      dfly_link* dlink = outgoing_adjacency_list_[src][i];
      switch_id dst = dlink->get_dst();
      assert(dlink->get_src_outport() == i);
      if (group_from_swid(src) == group_from_swid(dst))
        topology::setup_port_params(dlink->get_src_outport(), credits, electrical_bw, link_params, switch_params);
      else
        topology::setup_port_params(dlink->get_src_outport(), 100*credits, optical_bw, link_params, switch_params);

      //std::string portname = "port";
      //sprockit::sim_parameters* port_param = switch_params->get_namespace(portname + std::to_string(i));
      //double bw = port_param->get_bandwidth_param("bandwidth");
      //std::cout << "port" << std::to_string(i) << ": " << std::to_string(bw) << std::endl;

    }
    
    return;
  };

  void exacomm_dragonfly_topology::configure_nonuniform_switch_params(switch_id src, sprockit::sim_parameters* switch_params) const {
    std::string model_name = switch_params->get_optional_param("model", "pisces");
    if (!model_name.compare("dragonfly_switch")) {
      switch_params->add_param_override("id", int(src));
      switch_params->add_param_override("switches_per_group", int(switches_per_group_));
      switch_params->add_param_override("num_groups", int(num_groups_));
    } else if (!model_name.compare("pisces")) { 
      // still need to figure out what to fill in here
    } else if (!mode_name.compare("sculpin")) {

    }
    return;
  };

  /**
   * Configures the actual topology by populating the connected_outports and connected_inports 
   * of the topology
   **/
  void exacomm_dragonfly_topology::form_topology(std::string filename) { 
    std::ifstream mat_file(filename);
    int last_used_inport[max_switch_id_ + 1];
    int last_used_outport[max_switch_id_ + 1];
    std::memset(&last_used_outport, 0, (max_switch_id_ + 1) * sizeof(int));
    std::memset(&last_used_inport, 0, (max_switch_id_ + 1) * sizeof(int));
    if (mat_file.is_open()) {
      std::string line;
      std::getline(mat_file, line);
      int size = std::stoi(line, 0);
      assert(size == max_switch_id_ + 1);
      int i = 0;
      while ( std::getline(mat_file, line) ) {
        char* dup = (char *) line.c_str();
        const char* entry = std::strtok(dup, " ");
        int j = 0;
        while (entry!= NULL) {
          int cnt = std::stoi(entry, 0);
          if (cnt > 0) {
            sstmac::hw::Link_Type ltype = Electrical;
            if (group_from_swid(i) != group_from_swid(j)) ltype = Optical;
            outgoing_adjacency_list_[i].push_back(new dfly_link(i, last_used_outport[i], j , last_used_inport[j], ltype));
            //incoming_adjacency_list_[j].push_back(new dfly_link(i, last_used_outport[i], j , last_used_inport[j], ltype));
            last_used_outport[i]++;
            last_used_inport[j]++;
          }
          entry = std::strtok(NULL, " ");
          j++;
        }
        i++;
      }
    } else {
      spkt_abort_printf("Cannot Open the adjacency_matrix file");
    }
  };

  /**
   * Routes a single switch with id src so that it populates it's own row
   * of the routing table. This uses a queue and so it is a BFS. This makes sure
   * that the traffic balancing can be used to do load balancing routing
   * NOTE: The routing table stores that number of available ways one switch can be gotten by from its neighbors 
   **/
  void exacomm_dragonfly_topology::route_minimal_individual_switch(switch_id src, int& max_dist) {
    bool visited[max_switch_id_ + 1];
    for (int i = 0; i <= max_switch_id_; i++) {
      visited[i] = false;
    }
    std::fill(distance_matrix_[src].begin(), distance_matrix_[src].end(), INT_MAX);
    distance_matrix_[src][src] = 0;
    switch_id curr_switch = src;
    std::queue<switch_id> queue;
    queue.push(src);
    while (!queue.empty()) {
      curr_switch = queue.front();
      queue.pop();
      visited[curr_switch] = true;
      for (auto link : outgoing_adjacency_list_[curr_switch]) {
        switch_id neighbor = link->get_dst();
        assert(link->get_src() == curr_switch);
        
        if (distance_matrix_[src][neighbor] > distance_matrix_[src][curr_switch] + 1) {
          distance_matrix_[src][neighbor] = distance_matrix_[src][curr_switch] + 1;
          routing_table_[src][neighbor] = curr_switch;
        } else if (load_balance_routing_ && ((distance_matrix_[src][neighbor] == distance_matrix_[src][curr_switch] + 1) && visited[neighbor])) {
          if (switch_usage_[routing_table_[src][neighbor]] > switch_usage_[curr_switch]) {
            assert(switch_usage_[routing_table_[src][neighbor]] > 0);
            switch_usage_[routing_table_[src][neighbor]]--;
            routing_table_[src][neighbor] = curr_switch;
            switch_usage_[curr_switch]++;
          }
        }
          
        if (!visited[neighbor]) {
          queue.push(neighbor);
        }  
      }
    }

    for (int i = 0; i <= max_switch_id_; i++) {
      if (distance_matrix_[src][i] > max_dist) {
        max_dist = distance_matrix_[src][i];
      }
    }
    return;
  }

  void exacomm_dragonfly_topology::route_minimal_topology(int& max_dist) {
    switch_usage_.resize(max_switch_id_ + 1);
    routing_table_.resize(max_switch_id_ + 1);
    max_dist = 0;
    std::fill(switch_usage_.begin(), switch_usage_.end(), 0);
    for (int i = 0; i <= max_switch_id_; i++ ) {
      routing_table_[i].resize(max_switch_id_ + 1);
      route_minimal_individual_switch(i, max_dist);
    }
    return;
  }

  void exacomm_dragonfly_topology::form_canonical_dragonfly() {
    int inports[max_switch_id_ + 1];
    int outports[max_switch_id_ + 1];
    std::memset(&inports, 0, sizeof(int) * (max_switch_id_ + 1));
    std::memset(&outports, 0, sizeof(int) * (max_switch_id_ + 1));
    // form intra-group links
    for (int group = 0; group < num_groups_; group++) {
      switch_id group_offset = group * switches_per_group_;
      for (int i = 0; i < switches_per_group_ - 1; i++) {
        switch_id src = group_offset + i;
        for (int j = i + 1; j < switches_per_group_; j++) {
          switch_id dst = group_offset + j;
          outgoing_adjacency_list_[src].push_back(new dfly_link(src, outports[src], dst , inports[dst], Electrical));
          //incoming_adjacency_list_[dst].push_back(new dfly_link(src, outports[src], dst , inports[dst], Electrical));
          outports[src]++;
          inports[dst]++;

          outgoing_adjacency_list_[dst].push_back(new dfly_link(dst, outports[dst], src , inports[src], Electrical));
          //incoming_adjacency_list_[src].push_back(new dfly_link(dst, outports[dst], src , inports[src], Electrical));
          outports[dst]++;
          inports[src]++;
        }
      }
    }
    switch_id last_group_used_switch[num_groups_];
    std::memset(&last_group_used_switch, 0, sizeof(switch_id) * num_groups_);
    for (int src_group = 0; src_group < num_groups_ - 1; src_group++) {
      //switch_id src = (src_group * switches_per_group_) + last_group_used_switch[src_group];
      for (int dst_group = src_group + 1; dst_group < num_groups_; dst_group++) {
        switch_id src = (src_group * switches_per_group_) + last_group_used_switch[src_group];
        switch_id dst = (dst_group * switches_per_group_) + last_group_used_switch[dst_group];

        outgoing_adjacency_list_[src].push_back(new dfly_link(src, outports[src], dst , inports[dst], Optical));
        //incoming_adjacency_list_[dst].push_back(new dfly_link(src, outports[src], dst , inports[dst], Optical));
        outports[src]++;
        inports[dst]++;

        outgoing_adjacency_list_[dst].push_back(new dfly_link(dst, outports[dst], src , inports[src], Optical));
        //incoming_adjacency_list_[src].push_back(new dfly_link(dst, outports[dst], src , inports[src], Optical));
        outports[dst]++;
        inports[src]++;

        last_group_used_switch[src_group]++;
        last_group_used_switch[dst_group]++;
      }
    }

  };
}
}


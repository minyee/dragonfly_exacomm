/**
 * Author: Min Yee Teh
 * This is the header file containing all the function footprint for the flexfly topology class
 * which is really a canonical Dragonfly
 */
#ifndef EXACOMM_DRAGONFLY_TOPOLOGY
#define EXACOMM_DRAGONFLY_TOPOLOGY


#include <sprockit/keyword_registration.h>
#include <algorithm>
#include <vector>
#include <sstmac/hardware/topology/topology.h>
#include <sstmac/hardware/topology/structured_topology.h>
#include "dfly_link.h"



namespace sstmac{
namespace hw {


class exacomm_dragonfly_topology : public structured_topology {
  FactoryRegister("exacomm_dragonfly", topology, exacomm_dragonfly_topology)
public:
//FactoryRegister("exacomm_dragonfly_topology", topology, exacomm_dragonfly_topology, "This is dragonfly topology for exacomm dragonfly design space exploration project");
/*
RegisterComponent("flexfly", topology, exacomm_dragonfly_topology,
           "topol", COMPONENT_CATEGORY_NETWORK,
           "The flexfly topology")
*/
virtual int diameter() const override {
  // all dragonfly topolgies will have to have a diameter of 3
  return diameter_;
};




public:
 virtual std::string to_string() const override {
  return "exacomm_dragonfly";
 };
 
  exacomm_dragonfly_topology(sprockit::sim_parameters* params);

  virtual ~exacomm_dragonfly_topology() override;
   /**** BEGIN PURE VIRTUAL INTERFACE *****/
  /**
   * @brief Whether all network ports are uniform on all switches,
   *        having exactly the same latency/bandwidth parameters.
   *        If a 3D torus, e.g., has X,Y,Z directions exactly the same,
   *        this returns true.
   * @return false cause the network ports are definitely not the same for all links
   */
  virtual bool uniform_network_ports() const override { //DONE
  	return false;
  };

  /**
   * @brief Whether all switches are the same, albeit with each port on the switch
   *        having slightly different latency/bandwidth configurations
   * @return
   */
  virtual bool uniform_switches_non_uniform_network_ports() const override{ //DONE
  	//no this is false because we have both electrical and optical switches
  	return false; 
  };

  /**
   * @brief Whether all switches are the same and all ports on those switches
   *        have exactly the same configuration
   * @return false because some switches may have 
   */
  virtual bool uniform_switches() const override { //DONE
  	return false;
  };

  /**
   * @brief connected_outports
   *        Given a 3D torus e.g., the connection vector would contain
   *        6 entries, a +/-1 for each of 3 dimensions.
   * @param src   Get the source switch in the connection
   * @param conns The set of output connections with dst switch_id
   *              and the port numbers for each connection
   */
  virtual void connected_outports(switch_id src,
                     std::vector<topology::connection>& conns) const override; //DONE

  /**
     For indirect networks, this includes all switches -
     those connected directly to nodes and internal
     switches that are only a part of the network
     @return The total number of switches
  */
  virtual int num_switches() const override { 
  	return num_groups_ * switches_per_group_;
  };

  /**
   * @brief max_switch_id Depending on the node indexing scheme, the maximum switch id
   *  might be larger than the actual number of switches.
   * @return The max switch id
   */
  virtual switch_id max_switch_id() const override{ // DONE
    return num_groups_ * switches_per_group_;
  };

  /**
   * @brief swithc_id_slot_filled
   * @param sid
   * @return Whether a switch object should be built for a given switch_id
   */
  virtual bool switch_id_slot_filled(switch_id sid) const override; // DONE

  virtual int num_nodes() const override { // DONE
    int node_num = num_groups_ * switches_per_group_ * nodes_per_switch_;
    //std::cout << "num_nodes: " << std::to_string(node_num) << std::endl;
    return node_num;
  };

  /**
   * @brief max_node_id Depending on the node indexing scheme, the maximum node id
   *  might be larger than the actual number of nodes.
   * @return The max node id
   */
  virtual node_id max_node_id() const override{ // DONE
    return (num_groups_ * switches_per_group_ * nodes_per_switch_) ;
  };

  /**
   * @brief node_id_slot_filled
   * @param nid
   * @return Whether a node object should be built for a given node_id
   */
  virtual bool node_id_slot_filled(node_id nid) const override { // DONE
    //std::cout << "node_id_slot_filled?" << std::endl;
    return (nid < exacomm_dragonfly_topology::max_node_id());
  };

  virtual switch_id max_netlink_id() const override; // DONE (RECHECK)

  virtual bool netlink_id_slot_filled(node_id nid) const override; // DONE (RECHECK)

  /**
   * @brief num_endpoints To be distinguished slightly from nodes.
   * Multiple nodes can be grouped together with a netlink.  The netlink
   * is then the network endpoint that injects to the switch topology
   * @return
   */
  virtual int num_netlinks() const override; // DONE (RECHECK)

  /**
   * @brief Return the maximum number of ports on any switch in the network
   * @return
   */
  virtual int max_num_ports() const override { // DONE (RECHECK)
  	//return std::max(optical_switch_radix_, electrical_switch_radix_);
    int max_port_cnt = 0;
    for (int i = 0; i <= max_switch_id_; i++) {
      if (max_port_cnt < (outgoing_adjacency_list_[i].size() + nodes_per_switch_)) {
        max_port_cnt = outgoing_adjacency_list_[i].size() + nodes_per_switch_;
      }
    }
    return max_port_cnt; // + 1 - 1
  }

  /**
     For a given node, determine the injection switch
     All messages from this node inject into the network
     through this switch
     @param nodeaddr The node to inject to
     @param switch_port [inout] The port on the switch the node injects on
     @return The switch that injects from the node
  */
  virtual switch_id netlink_to_injection_switch(
        netlink_id nodeaddr, uint16_t& switch_port) const override;

  /**
     For a given node, determine the ejection switch
     All messages to this node eject into the network
     through this switch
     @param nodeaddr The node to eject from
     @param switch_port [inout] The port on the switch the node ejects on
     @return The switch that ejects into the node
  */
  virtual switch_id netlink_to_ejection_switch(
        netlink_id nodeaddr, uint16_t& switch_port) const override;


  virtual void configure_individual_port_params(switch_id src,
          sprockit::sim_parameters* switch_params) const override;
  
  /**
   * @brief node_to_ejection_switch Given a destination node,
   *        figure out which switch has an ejection connection to it
   * @param addr
   * @param port  The port number on the switch that leads to ejection
   *              to the particular node
   * @return
   */
  virtual switch_id node_to_ejection_switch(node_id addr, uint16_t& port) const override; // DONE (RECHECK)

  virtual switch_id node_to_injection_switch(node_id addr, uint16_t& port) const override; // DONE (RECHECK)

  /**
    This gives the minimal distance counting the number of hops between switches.
    @param src. The source switch.
    @param dest. The destination switch.
    @return The number of hops to fibal destination
  */
  virtual int minimal_distance(switch_id src, switch_id dst) const override; // DONE (PLEASE RECHECK)
  	//return topology_diameter_;

  /**
    This gives the minimal distance counting the number of hops between switches.
    @param src. The source node.
    @param dest. The destination node.
    @return The number of hops to final destination
  */
  virtual int num_hops_to_node(node_id src, node_id dst) const override;

  /**
   * @brief configure_switch_params By default, almost all topologies
   *        have uniform switch parameters.
   * @param src
   * @param switch_params In/out parameter. Input is default set of params.
   *        Output is non-default unique params.
   */
  virtual void configure_nonuniform_switch_params(switch_id src,
        sprockit::sim_parameters* switch_params) const override;

  /**
     For a given input switch, return all nodes connected to it.
     This return vector might be empty if the
     switch is an internal switch not connected to any nodes
     @return The nodes connected to switch for injection
  */
  virtual void nodes_connected_to_injection_switch(switch_id swid,
                          std::vector<injection_port>& nodes) const override;

  /**
     For a given input switch, return all nodes connected to it.
     This return vector might be empty if the
     switch is an internal switch not connected to any nodes
     @return The nodes connected to switch for ejection
  */
  virtual void nodes_connected_to_ejection_switch(switch_id swid,
                          std::vector<injection_port>& nodes) const override;

  virtual int num_leaf_switches() const override {
    return num_groups_ * switches_per_group_;
  };
  /**
     Given the current location and a destination,
     compute the minimal path to the destination.
     The path generally consists of a dimension,
     a direction or branch along that dimension,
     a port number for that dim/dir combination,
     and a virtual channel along the dim/dir
     appropriate for avoiding deadlock.
     @param current_sw_addr The addr of the current switch
     @param dest_sw_addr The addr of the destination switch
     @param path [inout] A complete path descriptor to the destination switch
  */
  virtual void minimal_route_to_switch( //this is really the key
    switch_id current_sw_addr,
    switch_id dest_sw_addr,
    packet::path& path) const override;

  virtual bool node_to_netlink(node_id nid, node_id& net_id, int& offset) const override;

  virtual switch_id random_intermediate_switch(switch_id current, switch_id dest, uint32_t seed) override;

  bool is_global_port(switch_id swid, int outport) const;

private:
 uint32_t num_groups_; // equivalent to parameter g in Kim's paper
 
 uint32_t switches_per_group_; // equivalent to parameter a in Kim's paper
 
 uint32_t nodes_per_switch_; // equivalent to parameter p in Kim's paper

 uint32_t optical_links_per_switch_; // this is equivalent to h in Kim's paper

 uint32_t max_switch_id_;

 std::vector<std::vector<dfly_link *>> outgoing_adjacency_list_;

 //std::vector<std::vector<dfly_link *>> incoming_adjacency_list_;

 std::vector<std::vector<int>> distance_matrix_;

 std::vector<std::vector<switch_id>> routing_table_;

 std::vector<uint16_t> switch_usage_; // used to keep count how much each switch has been used for routing
                                      // this allows for uniform traffic for statis routing

 bool load_balance_routing_;

 int diameter_;

 bool valid_switch_id(switch_id id) const {
    return id < (switches_per_group_ * num_groups_);
 };

 void form_topology(std::string filename);

 void print_port_connection_for_switch(switch_id swid) const;

 void check_routing_table() const;

 void route_minimal_topology(int& max_dist);

 void route_minimal_individual_switch(switch_id src, int& max_dist);

 void form_canonical_dragonfly();

 std::vector<std::vector<std::vector<std::pair<switch_id, int>>>> group_to_group_connections_;

public:
 int num_groups() {
 	return num_groups_;
 };
 
 int switches_per_group() {
 	return switches_per_group_;
 };

 int nodes_per_switch() {
  return nodes_per_switch_;
 };

  // computes the switch id to which node id nid belongs to
 switch_id node_to_switch(node_id nid) const;

  // computes from the switch id the group that the switch id in the argument belongs to
 int group_from_swid (switch_id swid) const;

 void switches_connecting_groups(int src_group, 
                                  int dst_group, 
                                  std::vector<std::pair<switch_id, int>>& switch_port_pairs) const; 
};


}
}

#endif
#ifndef EXACOMM_DRAGONFLY_SWITCH 
#define EXACOMM_DRAGONFLY_SWITCH

#include <sstmac/hardware/switch/network_switch.h>
#include <sstmac/hardware/topology/topology.h>
#include <vector> 
#include "exacomm_dragonfly_topology.h"
#include <sstmac/hardware/common/connection.h>
#include <sstmac/common/sstmac_config.h>
#include <sstmac/common/sst_event.h>
#include <sprockit/keyword_registration.h>

//RegisterNamespaces("switch", "link", "xbar", "router");



namespace sstmac {
namespace hw {

//typedef int port_id;

class dragonfly_switch : public network_switch {
  DeclareFactory(dragonfly_switch,uint32_t,event_manager*)
public:

RegisterComponent("dragonfly_switch", network_switch, dragonfly_switch,
         			"macro", COMPONENT_CATEGORY_NETWORK,
         			"A typical dragonfly switch used in the exacomm dragonfly design space exploration project");
 
dragonfly_switch(sprockit::sim_parameters* params,
							uint64_t id,
							event_manager* mgr);// : network_switch(params, id, mgr, ty);
 
~dragonfly_switch();

virtual std::string to_string() const override {
	return "dragonfly_switch";
};

/**
 * @brief connect_output
 * @param params
 * @param src_outport
 * @param dst_inport
 * @param payload_handler
 */
virtual void connect_output(sprockit::sim_parameters* params, 
                                int src_outport, 
                                int dst_inport, 
                                event_link* payload_handler) override;


/**
 * @brief connect_input
 * @param params
 * @param src_outport
 * @param dst_inport
 * @param credit_handler Can be null, if no credits are ever sent
 */
virtual void connect_input(sprockit::sim_parameters* params, 
                              int src_outport, 
                              int dst_inport,
                              event_link* credit_handler) override;

/**
 * @brief credit_handler
 * @param port
 * @return Can be null, if no credits are ever to be received
 */
virtual link_handler* credit_handler(int port) const override;

/**
 * @brief payload_handler
 * @param port
 *  @return
 */
virtual link_handler* payload_handler(int port) const override;

virtual int queue_length(int port) const override;

virtual timestamp send_latency(sprockit::sim_parameters* params) const override;

virtual timestamp credit_latency(sprockit::sim_parameters* params) const override;

protected:

void recv_payload(event* ev);

void recv_credit(event* ev);

void recv_nodal_payload(event* ev);

void recv_nodal_credit(event* ev);

private:
std::vector<event_link*> switch_inport_handlers_;
std::vector<event_link*> switch_outport_handlers_;
std::vector<event_link*> nodal_inport_handlers_;
std::vector<event_link*> nodal_outport_handlers_;
int switch_radix_;
int* credits_nodal_;
int* credits_switch_;
switch_id my_addr_;
int switches_per_group_;
int nodes_per_switch_;
int num_groups_;
exacomm_dragonfly_topology* dtop_;

double optical_link_bw_;
double electrical_link_bw_;
timestamp inv_optical_link_bw_;
timestamp inv_electrical_link_bw_;
timestamp send_payload_latency_;
timestamp send_credit_latency_;

};
 
}
}

#endif
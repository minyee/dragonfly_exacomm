#include "dragonfly_switch.h"
#include <sstmac/hardware/common/connection.h>
#include <sprockit/sim_parameters.h>
#include <sprockit/util.h>
#include <sprockit/factories/factory.h>
#include <sstmac/hardware/pisces/pisces.h>
#include <sstmac/hardware/network/network_message.h>

MakeDebugSlot(dragonfly_switch)

/*
RegisterKeywords( "id", 
					"switches_per_group", 
					"switch_radix",
					"nodes_per_switch", 
					"electrical_link_bandwidth",
					"send_latency",
					"credit_latency",
				); 
*/

RegisterNamespaces(
	"link",
	"xbar",
	"router"
);

RegisterKeywords(
                  {"id","DEPRECATED: the id of this switch"},
                  {"switches_per_group", "The number of switches in every group of switches"},
                  {"switch_radix", "The number of ports that is used to connect to other switches"},
                  {"nodes_per_switch","The p parameter in Dragonfly literatures"},
                  {"electrical_link_bandwidth", "The bandwidth of the electrical links"},
                  {"optical_switch_bandwidth", "The bandwidth of the optical links"},
                  {"send_latency", ""},
                  {"credit_latency", ""},
);

namespace sstmac {
namespace hw {

	dragonfly_switch::dragonfly_switch(sprockit::sim_parameters* params,
    														uint64_t id,
    														event_manager* mgr) : 
																network_switch(params, 
																				id,
																				mgr) {

		my_addr_ = params->get_int_param("id");
		switches_per_group_ = params->get_int_param("switches_per_group");
		//num_groups_ = params->get_int_param("groups");
		switch_radix_ = params->get_int_param("switch_radix");
		nodes_per_switch_ = params->get_int_param("nodes_per_switch");
		sprockit::sim_parameters* link_params = params->get_namespace("link");
		electrical_link_bw_ = link_params->get_bandwidth_param("electrical_link_bandwidth");
		optical_link_bw_ = link_params->get_bandwidth_param("electrical_link_bandwidth");
		inv_optical_link_bw_ = 1 / optical_link_bw_;
		inv_electrical_link_bw_ = 1 / electrical_link_bw_;
		send_payload_latency_ = link_params->get_time_param("send_latency");
		send_credit_latency_ = link_params->get_time_param("credit_latency");
		credits_nodal_ = new int[nodes_per_switch_]; 
		credits_switch_ = new int[switch_radix_]; 
		std::memset(credits_nodal_, 0, sizeof(int) * nodes_per_switch_);
		std::memset(credits_switch_, 0, sizeof(int) * switch_radix_);
		// Initiate the outport handlers and their corresponding outport switches
		switch_inport_handlers_.resize(switch_radix_);
		switch_outport_handlers_.resize(switch_radix_);
		nodal_inport_handlers_.resize(nodes_per_switch_);
		nodal_outport_handlers_.resize(nodes_per_switch_);
		for (int i = 0; i < switch_radix_; i++) {
			switch_outport_handlers_[i] = nullptr;
			switch_inport_handlers_[i] = nullptr;
		}
		for (int i = 0; i < nodes_per_switch_; i++) {
			nodal_inport_handlers_[i] = nullptr;
			nodal_outport_handlers_[i] = nullptr;
		}
		dtop_ = dynamic_cast<exacomm_dragonfly_topology *>(topology::static_topology(params));
		init_links(params);
	}

	dragonfly_switch::~dragonfly_switch() {
		if (credits_switch_ != nullptr)
			delete [] credits_nodal_;
		if (credits_nodal_ != nullptr) 
			delete [] credits_switch_;
	}

	int dragonfly_switch::queue_length(int port) const {
		return 0;
	}

	void dragonfly_switch::connect_input(sprockit::sim_parameters* params, 
                              						int src_outport, 
                              						int dst_inport,
                              						event_link* credit_handler) {
		if (dst_inport < switch_radix_) {
			switch_inport_handlers_[dst_inport] = credit_handler;
		} else if (switch_radix_ <= dst_inport) {
			nodal_inport_handlers_[dst_inport - switch_radix_] = credit_handler;
		} else {
			spkt_abort_printf("Invalid connect inport for switch with id: %d", my_addr_);
		}
	}

	void dragonfly_switch::connect_output(sprockit::sim_parameters* params, 
                              						int src_outport, 
                              						int dst_inport,
                              						event_link* payload_handler) {
		if (src_outport < switch_radix_) {
			switch_outport_handlers_[src_outport] = payload_handler;
		} else if (src_outport <= switch_radix_ + nodes_per_switch_ - 1) {
			nodal_outport_handlers_[src_outport - switch_radix_] = payload_handler;
		} else {
			spkt_abort_printf("Invalid connect outport for switch with id: %d", my_addr_);
		}
	}

	link_handler* dragonfly_switch::credit_handler(int port) const {
		if (port < switch_radix_) {
			return new_link_handler(this, &dragonfly_switch::recv_credit);
		} else {
			return new_link_handler(this, &dragonfly_switch::recv_nodal_credit);
		}
	}

	link_handler* dragonfly_switch::payload_handler(int port) const {
		//std::cout << "Payload Handler for switch id : " << std::to_string(my_addr_) << " for port : " << std::to_string(port) << std::endl;
		if (port < switch_radix_) {
			return new_link_handler(this, &dragonfly_switch::recv_payload);
		} else {
			return new_link_handler(this, &dragonfly_switch::recv_nodal_payload);
		}
	}

	/**
	 * Receiving a packet or message from another switch, not from a node.
	 **/
	void dragonfly_switch::recv_payload(event* ev) { 
  		pisces_default_packet* msg = safe_cast(pisces_default_packet, ev);
  		int dst = msg->toaddr();
		int dst_switch = dtop_->node_to_switch(dst);
		int dst_group = dtop_->group_from_swid(dst_switch);
		int outport;
		timestamp actual_delay = send_payload_latency_ + (8 * msg->num_bytes()) * inv_electrical_link_bw_;
		//timestamp optical_delay = send_payload_latency_ + (8 * msg->num_bytes()) * inv_optical_link_bw_;
		event_link* eh; 
		if (dst_switch == my_addr_) {
			outport = dst % nodes_per_switch_;
			eh = nodal_outport_handlers_[outport];
			//send_delayed_to_link(electrical_delay, nodal_outport_handlers_[outport], ev);
		} else {
			routable::path pth;
			dtop_->minimal_route_to_switch(my_addr_, dst_switch, pth);
			outport = pth.outport();
			if (outport >= switch_outport_handlers_.size())
				spkt_abort_printf("RECV_NODAL_PAYLOAD cannot find valid outport at %d \n", outport);
			eh = switch_outport_handlers_[outport];
			if (dst_group != dtop_->group_from_swid(my_addr_)) {
				actual_delay = send_payload_latency_ + (8 * msg->num_bytes()) * inv_optical_link_bw_;	
			} 
		}
		if (eh == nullptr)
			spkt_abort_printf("RECV_PAYLOAD cannot recv_payload, unmapped\n");
		eh->send_extra_delay(actual_delay, ev);
		return;
	}

	/**
	 * Receiving a packet or message from a node, not from a switch.
	 * This can be said to be an injection packet. Must be routed through
	 * the network 
	 **/
	void dragonfly_switch::recv_nodal_payload(event* ev) {
		pisces_default_packet* msg = safe_cast(pisces_default_packet, ev);
		int src = msg->fromaddr();
		int dst = msg->toaddr();
		int src_switch = dtop_->node_to_switch(src);
		int dst_switch = dtop_->node_to_switch(dst);
		int src_group = dtop_->group_from_swid(src_switch);
		int dst_group = dtop_->group_from_swid(dst_switch);
		timestamp actual_delay = send_payload_latency_ + (8 * msg->num_bytes()) * inv_electrical_link_bw_;
		int outport; 
		event_link* eh = nullptr; 
		pisces_credit* cred = new pisces_credit(0, 0, 100000);
		nodal_inport_handlers_[src % nodes_per_switch_]->send_extra_delay(send_credit_latency_, cred);
		if (my_addr_ == dst_switch) {
			outport = dst % nodes_per_switch_;
			eh = nodal_outport_handlers_[outport];
		} else {
			routable::path pth;
			dtop_->minimal_route_to_switch(my_addr_, dst_switch, pth); // CONTINUE HERE, THIS IS WHERE IT FAILS
			outport = pth.outport();
			if (outport >= switch_outport_handlers_.size())
				spkt_abort_printf("RECV_NODAL_PAYLOAD cannot find valid outport at %d \n", outport);
			eh = switch_outport_handlers_[outport];
			if (dst_group != src_group) {
				actual_delay = send_payload_latency_ + (8 * msg->num_bytes()) * inv_optical_link_bw_;	
			} 
		} 
		assert(eh != nullptr);
		eh->send_extra_delay(actual_delay, ev);
		//send_delayed_to_link(actual_delay, eh, ev);
	};

	void dragonfly_switch::recv_nodal_credit(event* ev) {
		pisces_credit* cred = safe_cast(pisces_credit, ev);
		credits_nodal_[cred->port()] += cred->num_credits();
		return;
	};

	void dragonfly_switch::recv_credit(event* ev) {
		pisces_credit* cred = safe_cast(pisces_credit, ev);
		credits_switch_[cred->port()] += cred->num_credits();
		return;
	};

	timestamp dragonfly_switch::send_latency(sprockit::sim_parameters* params) const {
		return send_payload_latency_;
	}

	timestamp dragonfly_switch::credit_latency(sprockit::sim_parameters* params) const {
		return send_credit_latency_;	
	}

	//void dragonfly_switch::send_credit() {

	//}
}
}
#ifndef EXACOMM_DRAGONFLY_UGAL_ROUTING_H
#define EXACOMM_DRAGONFLY_UGAL_ROUTING_H

#include <sstmac/hardware/router/ugal_routing.h>
#include <sstmac/hardware/router/router.h>
#include <sstmac/hardware/router/valiant_routing.h>
#include <sstmac/hardware/topology/topology.h>
#include <sstmac/hardware/interconnect/interconnect.h>

#include "exacomm_dragonfly_topology.h" 





namespace sstmac {
namespace hw {

/**
 * @brief The ugal_router class
 * Encapsulates a router that performs Univeral Globally Adaptive Load-balanced
 * routing as described in PhD Thesis "Load-balanced in routing in interconnection networks"
 * by A Singh.
 */
class exacomm_dragonfly_ugal_router : public ugal_router
{

  static const char initial = 0;
  static const char valiant_stage = 1;
  static const char minimal_stage = 2;   
  static const char final_stage = 3;

  struct header : public ugal_router::header {
     char num_hops : 3;
     char num_group_hops : 3;
     char stage: 3;
  };

  public:


  //static const char final_stage = 3; // not sure if this is needed
  
  FactoryRegister("exacomm_dragonfly_ugal", router, exacomm_dragonfly_ugal_router,
              "router implementing ugal congestion-aware routing in the exacomm_dragonfly")

  exacomm_dragonfly_ugal_router(sprockit::sim_parameters* params, topology* top, network_switch* netsw);
  

  std::string to_string() const override {
    return "exacomm_dragonfly_ugal";
  };

  virtual int num_vc() const override {
    return 3;
  }

  virtual void route(packet* pkt) override;

 protected:
  int val_threshold_;
  int val_preference_factor_;
 
 private:
  bool switch_paths(switch_id orig_dst, switch_id new_dst, packet::path& orig_path, packet::path& new_path);

  bool route_common(packet* pkt);

  void route_initial(packet* pkt, switch_id ej_addr);

  exacomm_dragonfly_topology* dtop_;

  uint32_t seed_;
};

}
}

#endif // UGAL_ROUTING_H
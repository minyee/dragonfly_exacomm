#ifndef EXACOMM_DRAGONFLY_UGAL_ROUTING_H
#define EXACOMM_DRAGONFLY_UGAL_ROUTING_H

#include <sstmac/hardware/interconnect/interconnect.h>
#include <sstmac/hardware/router/ugal_routing.h>
#include <sstmac/hardware/router/router.h>
#include <sstmac/hardware/router/valiant_routing.h>

#include "exacomm_dragonfly_topology.h" 


namespace sstmac {
namespace hw {

/**
 * @brief The ugal_router class
 * Encapsulates a router that performs Univeral Globally Adaptive Load-balanced
 * routing as described in PhD Thesis "Load-balanced in routing in interconnection networks"
 * by A Singh.
 */
class exacomm_dragonfly_ugalG_router : public ugal_router
{
  struct header : public ugal_router::header {
    char stage : 3;
    uint16_t entrySWID;
    uint16_t exitSWID;
  };
  public:
  FactoryRegister("exacomm_dragonfly_simplified_ugalG", router, exacomm_dragonfly_ugalG_router,
              "router implementing ugal global-congestion-aware routing in the exacomm_dragonfly_topology")
  exacomm_dragonfly_ugalG_router(sprockit::sim_parameters* params, topology* top, network_switch* netsw);
  

  std::string to_string() const override {
    return "exacomm_dragonfly_simplified_ugal";
  };




  virtual int num_vc() const override {
    return 3;
  }


  virtual void route(packet* pac) override;

 protected:
  int val_threshold_;
  int val_preference_factor_;
 
 private:
  void route_to_intermediate_group_stage(packet* pkt);

  void route_to_dest(packet* pkt);

  exacomm_dragonfly_topology* dtop_;
  
  hw::interconnect* ic_;
};  

}
}


#endif // UGAL_ROUTING_H
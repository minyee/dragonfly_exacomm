#ifndef EXACOMM_DRAGONFLY_UGAL_ROUTING_H
#define EXACOMM_DRAGONFLY_UGAL_ROUTING_H

#include <sstmac/hardware/router/ugal_routing.h>
#include <sstmac/hardware/router/router.h>
#include <sstmac/hardware/router/valiant_routing.h>
#include "exacomm_dragonfly_topology_simplified.h"
namespace sstmac {
namespace hw {

/**
 * @brief The ugal_router class
 * Encapsulates a router that performs Univeral Globally Adaptive Load-balanced
 * routing as described in PhD Thesis "Load-balanced in routing in interconnection networks"
 * by A Singh.
 */
class exacomm_dragonfly_minimal_router :
  public router
{
  FactoryRegister("exacomm_dragonfly_minimal", router, exacomm_dragonfly_minimal_router,
              "router implementing minimal routing in the exacomm dragonfly topology")
  exacomm_dragonfly_minimal_router(sprockit::sim_parameters* params, topology* top, network_switch* netsw);
 public:
  

  std::string to_string() const override {
    return "exacomm_dragonfly_minimal";
  };

  void route(packet* pack) override;

 protected:


  virtual int num_vc() const override {
    return 2;
  }

 protected:

 private:
  exacomm_dragonfly_topology* dtop_;
  switch_id sid_;
};

}
}


#endif // UGAL_ROUTING_H
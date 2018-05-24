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

  static const char initial_stage = 0;
  static const char hop_to_entry_stage = 1;
  static const char hop_to_exit_stage = 2;
  static const char final_stage = 3;
  static const char intra_grp_stage = 4;

  struct header : public ugal_router::header {
    uint8_t num_group_hops : 2;
    uint16_t entry_swid;
    uint16_t exit_swid;
    uint16_t intermediate_grp;
  };
  public:
  FactoryRegister("exacomm_dragonfly_ugalG", router, exacomm_dragonfly_ugalG_router,
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
  void select_ugalG_intermediate(packet* pkt, switch_id ej_addr); 

  void find_min_group_link(int src_grp, int dst_grp, switch_id& swid, int& queue_length);

  exacomm_dragonfly_topology* dfly_;
  
  hw::interconnect* ic_;
};  

}
}


#endif // UGAL_ROUTING_H
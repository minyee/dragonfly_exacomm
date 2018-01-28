#ifndef DFLY_LINK_TYPES
#define DFLY_LINK_TYPES

namespace sstmac {
namespace hw {

enum Link_Type {Optical, Electrical};

class dfly_link {
	public:
	dfly_link(switch_id src_sid, uint8_t src_outport, switch_id dst_sid, uint8_t dst_inport, Link_Type type) :
		src_sid_(src_sid), 
		dst_sid_(dst_sid),
		src_outport_(src_outport),
		dst_inport_(dst_inport),
		type_(type)
	{}

	uint8_t get_src_outport() const {
		return src_outport_;
	}

	uint8_t get_dst_inport() const {
		return dst_inport_;
	}

	switch_id get_dst() const {
		return dst_sid_; 
	}

	switch_id get_src() const {
		return src_sid_;
	}

	Link_Type get_link_type() const {
		return type_;
	}

	private:
	uint8_t src_outport_;
	uint8_t dst_inport_;
	switch_id src_sid_;
	switch_id dst_sid_;
	Link_Type type_;
};

}
}

#endif
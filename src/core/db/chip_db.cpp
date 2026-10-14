#include "chip_db.hpp"
#include <stdexcept>

namespace soc::db {

ChipDB::ChipDB() {
    design_name = strings.intern("top_chip");
}

LayerId ChipDB::create_layer(std::string_view name, uint16_t gds_layer, uint16_t gds_datatype,
                             int32_t pitch, int32_t min_width, int32_t min_spacing,
                             bool is_routing, bool is_cut) {
    StringId sname = strings.intern(name);
    uint32_t id_val = layers.emplace_back();
    Layer& lyr = layers[id_val];
    lyr.id = LayerId{id_val};
    lyr.name_id = sname;
    lyr.gds_layer = gds_layer;
    lyr.gds_datatype = gds_datatype;
    lyr.pitch = pitch;
    lyr.min_width = min_width;
    lyr.min_spacing = min_spacing;
    lyr.is_routing = is_routing;
    lyr.is_cut = is_cut;
    return lyr.id;
}

MasterCellId ChipDB::create_master_cell(std::string_view name, int32_t width, int32_t height,
                                        CellType type) {
    StringId sname = strings.intern(name);
    uint32_t id_val = master_cells.emplace_back();
    MasterCell& cell = master_cells[id_val];
    cell.id = MasterCellId{id_val};
    cell.name_id = sname;
    cell.width = width;
    cell.height = height;
    cell.cell_type = type;
    return cell.id;
}

InstanceId ChipDB::create_instance(std::string_view name, MasterCellId master_id,
                                   Point2D origin, Orientation orient,
                                   PowerDomainId domain, BlockAbstractId abstract_id) {
    StringId sname = strings.intern(name);
    uint32_t id_val = instances.emplace_back();
    Instance& inst = instances[id_val];
    inst.id = InstanceId{id_val};
    inst.name_id = sname;
    inst.master_id = master_id;
    inst.origin = origin;
    inst.orientation = orient;
    inst.power_domain_id = domain;
    inst.parent_abstract_id = abstract_id;

    if (master_id.is_valid() && master_id.value() < master_cells.size()) {
        const MasterCell& m = master_cells[master_id];
        Rect2D bbox = inst.compute_bbox(m.width, m.height);
        instance_rtree.insert(bbox, inst.id);
    }

    if (domain.is_valid() && domain.value() < power_domains.size()) {
        power_domains[domain].member_instances.push_back(inst.id);
    }

    if (abstract_id.is_valid() && abstract_id.value() < block_abstracts.size()) {
        block_abstracts[abstract_id].internal_instances.push_back(inst.id);
    }

    observers.notify_instance_added(inst.id);
    return inst.id;
}

void ChipDB::move_instance(InstanceId id, Point2D new_pos) {
    if (!id.is_valid() || id.value() >= instances.size()) return;
    Instance& inst = instances[id];
    Point2D old_pos = inst.origin;
    inst.origin = new_pos;
    observers.notify_instance_moved(id, old_pos, new_pos);
}

NetId ChipDB::create_net(std::string_view name, SignalType signal_type, PowerDomainId domain) {
    StringId sname = strings.intern(name);
    uint32_t id_val = nets.emplace_back();
    Net& net = nets[id_val];
    net.id = NetId{id_val};
    net.name_id = sname;
    net.signal_type = signal_type;
    net.power_domain_id = domain;

    if (domain.is_valid() && domain.value() < power_domains.size()) {
        power_domains[domain].internal_nets.push_back(net.id);
    }
    return net.id;
}

PinId ChipDB::create_pin(std::string_view name, InstanceId inst_id, NetId net_id,
                         LayerId layer, Rect2D offset_box, SignalType sig_type) {
    StringId sname = strings.intern(name);
    uint32_t id_val = pins.emplace_back();
    Pin& pin = pins[id_val];
    pin.id = PinId{id_val};
    pin.name_id = sname;
    pin.instance_id = inst_id;
    pin.net_id = net_id;
    pin.layer_id = layer;
    pin.offset_box = offset_box;
    pin.signal_type = sig_type;

    if (net_id.is_valid() && net_id.value() < nets.size()) {
        nets[net_id].pins.push_back(pin.id);
    }
    return pin.id;
}

PowerDomainId ChipDB::create_power_domain(std::string_view name, double voltage, Rect2D boundary) {
    StringId sname = strings.intern(name);
    uint32_t id_val = power_domains.emplace_back();
    PowerDomain& pd = power_domains[id_val];
    pd.id = PowerDomainId{id_val};
    pd.name_id = sname;
    pd.nominal_voltage = voltage;
    pd.boundary = boundary;
    return pd.id;
}

BlockAbstractId ChipDB::create_block_abstract(std::string_view name, std::string_view macro_type,
                                              Rect2D boundary, PowerDomainId domain) {
    StringId sname = strings.intern(name);
    uint32_t id_val = block_abstracts.emplace_back();
    BlockAbstract& ba = block_abstracts[id_val];
    ba.id = BlockAbstractId{id_val};
    ba.name_id = sname;
    ba.macro_type = std::string(macro_type);
    ba.boundary = boundary;
    ba.power_domain_id = domain;
    return ba.id;
}

Rect2D ChipDB::get_instance_bbox(InstanceId inst_id) const {
    if (!inst_id.is_valid() || inst_id.value() >= instances.size()) {
        return Rect2D{0, 0, 0, 0};
    }
    const Instance& inst = instances[inst_id];
    if (!inst.master_id.is_valid() || inst.master_id.value() >= master_cells.size()) {
        return Rect2D{inst.origin.x, inst.origin.y, inst.origin.x, inst.origin.y};
    }
    const MasterCell& m = master_cells[inst.master_id];
    return inst.compute_bbox(m.width, m.height);
}

void ChipDB::rebuild_instance_spatial_index() {
    instance_rtree.clear();
    for (uint32_t i = 0; i < instances.size(); ++i) {
        InstanceId id{i};
        Rect2D bbox = get_instance_bbox(id);
        instance_rtree.insert(bbox, id);
    }
}

std::vector<InstanceId> ChipDB::query_instances_in_box(const Rect2D& search_box) const {
    std::vector<InstanceId> results;
    instance_rtree.query_intersect(search_box, results);
    return results;
}

void ChipDB::query_instances_lod(const Rect2D& search_box, int32_t min_pixel_size_dbu,
                                 std::vector<InstanceId>& visible_instances,
                                 std::vector<Rect2D>& aggregated_clusters) const {
    instance_rtree.query_lod(search_box, min_pixel_size_dbu, visible_instances, aggregated_clusters);
}

void ChipDB::clear() {
    master_cells.clear();
    instances.clear();
    nets.clear();
    pins.clear();
    power_domains.clear();
    block_abstracts.clear();
    layers.clear();
    tracks.clear();
    vias.clear();
    micro_bumps.clear();
    nano_tsvs.clear();
    instance_rtree.clear();
    layer_rtrees.clear();
    strings.clear();
    observers.notify_cleared();
}

} // namespace soc::db

#include "mapper/mapper_ioctree.h"

namespace mapping {

I_OctreeMapper::I_OctreeMapper(const Options &options) : options_(options) {}

void I_OctreeMapper::Update(const MapperInput &) {}

void I_OctreeMapper::Reset() {}

bool I_OctreeMapper::Save(const std::string &) const { return false; }

const octomap::OcTree *I_OctreeMapper::GetOctree() const { return nullptr; }

} // namespace mapping
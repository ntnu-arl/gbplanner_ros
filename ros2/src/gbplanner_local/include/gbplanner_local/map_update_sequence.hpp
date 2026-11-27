#pragma once

#include <cstdint>
#include <set>
#include <stdexcept>
#include <string>
#include <utility>

namespace gbplanner2 {

// Metadata is checked before parsing a map and committed only after the whole
// map is accepted. Within a mapper instance, reset epochs and publication IDs
// order delayed service/topic samples, including epochs never received before.
// Artificial updates cannot manufacture a sensor observation.
class MapUpdateSequence {
 public:
  struct Update {
    std::string source;
    uint64_t epoch{0};
    uint64_t revision{0}, observation{0};
    int64_t observation_stamp_ns{0};
    std::string kind;
  };
  bool newer(const Update& next)const {
    static const std::set<std::string> kinds{
      "observation","freespace","augmentation","import","load","reset","snapshot"};
    if(next.source.empty()||next.source.size()>128||next.epoch==0||next.revision==0||
       next.observation>next.revision||next.observation_stamp_ns<0||!kinds.count(next.kind)||
       (next.observation==0&&next.observation_stamp_ns!=0))
      throw std::invalid_argument("Invalid map update metadata");
    if(retired_.count(next.source))return false;
    if(next.source!=current_.source)return true;
    if(next.revision<=current_.revision||next.epoch<current_.epoch)return false;
    // Replacing the map starts new sensor provenance, while the mapper's
    // publication ID continues across reset/load/import operations.
    if(next.epoch>current_.epoch)return true;
    // Different sensor queues can integrate a newer observation count with
    // an older acquisition time. Keep that real stamp for freshness checks;
    // only a snapshot retaining the same observation must retain its stamp.
    if(next.observation<current_.observation||
       (next.observation==current_.observation&&
        next.observation_stamp_ns!=current_.observation_stamp_ns))
      throw std::invalid_argument("Regressing map observation metadata");
    return true;
  }
  bool hasNewObservation(const Update& next)const {
    return next.observation>0&&
      (next.source!=current_.source||next.epoch>current_.epoch||
       next.observation>current_.observation);
  }
  void commit(Update next){
    if(!current_.source.empty()&&next.source!=current_.source)retired_.insert(current_.source);
    current_=std::move(next);
  }
  const Update& current()const{return current_;}

 private:
  Update current_;
  std::set<std::string> retired_;
};
}  // namespace gbplanner2

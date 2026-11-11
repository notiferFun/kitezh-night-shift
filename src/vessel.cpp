#include "kitezh/vessel.hpp"
#include <stdexcept>

namespace kitezh {

std::optional<std::string> validateVessel(const Vessel& vessel) {
    if (vessel.call_sign.empty()) {
        return "empty call sign";
    }
    for (char c : vessel.call_sign) {
        if (c == ' ' || c == '\t' || c == '\n') {
            return "bad call sign";
        }
    }
    if (vessel.length_m <= 0 || vessel.length_m > kChamberLengthM) {
        return "bad length";
    }
    if (vessel.draft_dm <= 0) {
        return "bad draft";
    }
    return std::nullopt;
}

bool fitsInChamber(const std::vector<Vessel>& inside, const Vessel& next) {
    if (next.length_m <= 0) {
        return false;
    }
    int total_length = next.length_m;
    for (const Vessel& v : inside) {
        if (v.length_m <= 0) {
            return false;
        }
        total_length += v.length_m + kGapM;
    }
    return total_length <= kChamberLengthM;
}

}  // namespace kitezh

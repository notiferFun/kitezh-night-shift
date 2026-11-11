#include "kitezh/lock.hpp"

#include <algorithm>
#include <stdexcept>
#include <string>
#include <vector>

#include "kitezh/geometry.hpp"

namespace kitezh {
namespace {

GateState& gateRef(Gate g, GateState& lower, GateState& upper) {
    return (g == Gate::Lower) ? lower : upper;
}

int depthForLevel(Level lvl) {
    return (lvl == Level::Low) ? kLowDepthDm : kHighDepthDm;
}

Level oppositeLevel(Level lvl) {
    return (lvl == Level::Low) ? Level::High : Level::Low;
}

bool hasVesselWithCallSign(const std::vector<Vessel>& vessels,
                           const std::string& call_sign) {
    return std::any_of(vessels.begin(), vessels.end(),
                       [&](const Vessel& v) { return v.call_sign == call_sign; });
}

bool hasOversized(const std::vector<Vessel>& vessels) {
    return std::any_of(vessels.begin(), vessels.end(),
                       [](const Vessel& v) { return v.kind == VesselKind::Oversized; });
}

}  // namespace

Lock::Lock() = default;

Level Lock::level() const { return level_; }

GateState Lock::gate(Gate g) const {
    return (g == Gate::Lower) ? lower_gate_ : upper_gate_;
}

int Lock::pumpCycles() const { return pump_cycles_; }

std::vector<Vessel> Lock::inside() const { return inside_; }

void Lock::open(Gate g) {
    GateState& target = gateRef(g, lower_gate_, upper_gate_);
    if (target == GateState::Open) {
        return;
    }

    const bool isLower = (g == Gate::Lower);
    const Level requiredLevel = isLower ? Level::Low : Level::High;
    const GateState otherGate = isLower ? upper_gate_ : lower_gate_;

    if (level_ != requiredLevel || otherGate == GateState::Open) {
        throw LockError(isLower ? "cannot open lower gate"
                                : "cannot open upper gate");
    }

    target = GateState::Open;
}

void Lock::close(Gate g) {
    gateRef(g, lower_gate_, upper_gate_) = GateState::Closed;
}

void Lock::pump() {
    if (lower_gate_ == GateState::Open || upper_gate_ == GateState::Open) {
        throw LockError("cannot pump with open gates");
    }

    const Level next = oppositeLevel(level_);
    const int nextDepth = depthForLevel(next);

    for (const Vessel& v : inside_) {
        if (v.draft_dm > nextDepth) {
            throw LockError("vessel draft exceeds water depth at target level");
        }
    }

    level_ = next;
    ++pump_cycles_;
}

void Lock::enter(const Vessel& vessel) {
    if (validateVessel(vessel).has_value()) {
        throw LockError("invalid vessel data");
    }

    if (hasVesselWithCallSign(inside_, vessel.call_sign)) {
        throw LockError("vessel with this call sign is already inside");
    }

    const bool upstream = (vessel.direction == Direction::Upstream);
    const GateState requiredGate = upstream ? lower_gate_ : upper_gate_;
    if (requiredGate != GateState::Open) {
        throw LockError(upstream ? "lower gate must be open for upstream entry"
                                 : "upper gate must be open for downstream entry");
    }

    if (vessel.draft_dm > depthForLevel(level_)) {
        throw LockError("vessel draft exceeds current water depth");
    }

    if (vessel.kind == VesselKind::Oversized && !inside_.empty()) {
        throw LockError("oversized vessel can only enter empty chamber");
    }
    if (hasOversized(inside_)) {
        throw LockError("chamber already contains an oversized vessel");
    }

    if (!fitsInChamber(inside_, vessel)) {
        throw LockError("vessel does not fit chamber length");
    }

    inside_.push_back(vessel);
}

void Lock::exit(const std::string& call_sign) {
    if (inside_.empty()) {
        throw LockError("chamber is empty");
    }

    const Vessel& head = inside_.front();
    if (head.call_sign != call_sign) {
        throw LockError("only the head vessel can exit");
    }

    const bool upstream = (head.direction == Direction::Upstream);
    const GateState requiredGate = upstream ? upper_gate_ : lower_gate_;
    if (requiredGate != GateState::Open) {
        throw LockError(upstream ? "upper gate must be open for upstream exit"
                                 : "lower gate must be open for downstream exit");
    }

    inside_.erase(inside_.begin());
}

}  // namespace kitezh
#include "kitezh/shift.hpp"
#include "kitezh/geometry.hpp"
#include "kitezh/lock.hpp"
#include <stdexcept>
#include <unordered_set>
#include <vector>

namespace kitezh {

    ShiftReport runNightShift(const std::vector<Vessel>& queue) {
    ShiftReport report;
    std::unordered_set<std::string> seen;
    std::vector<Vessel> waiting;

    // 1. Первичная фильтрация отказов
    for (const auto& vessel : queue) {
        if (seen.find(vessel.call_sign) != seen.end()) {
            report.refused.push_back({vessel.call_sign, "duplicate"});
            continue;
        }
        seen.insert(vessel.call_sign);

        if (validateVessel(vessel).has_value()) {
            report.refused.push_back({vessel.call_sign, "invalid"});
            continue;
        }

        if (vessel.draft_dm > kLowDepthDm) {
            report.refused.push_back({vessel.call_sign, "draft"});
            continue;
        }

        waiting.push_back(vessel);
    }

    Lock lock;

    // 2. Проводка партий
    while (!waiting.empty()) {
        Direction dir = waiting.front().direction;
        std::vector<Vessel> batch;
        std::vector<size_t> picked_indices;
        bool skipped = false;

        if (waiting.front().kind == VesselKind::Oversized) {
            batch.push_back(waiting.front());
            picked_indices.push_back(0);
        } else {
            for (size_t i = 0; i < waiting.size(); ++i) {
                const auto& cand = waiting[i];
                if (cand.direction != dir || cand.kind == VesselKind::Oversized || !fitsInChamber(batch, cand)) {
                    skipped = true;
                    continue;
                }

                if (!skipped || cand.kind == VesselKind::Ambulance) {
                    batch.push_back(cand);
                    picked_indices.push_back(i);
                } else {
                    skipped = true;
                }
            }
        }

        // Удаляем взятые суда из очереди ожидания с конца
        for (auto it = picked_indices.rbegin(); it != picked_indices.rend(); ++it) {
            waiting.erase(waiting.begin() + *it);
        }

        // 3. Выполнение шлюзования
        Level target_in_level = (dir == Direction::Upstream) ? Level::Low : Level::High;
        Gate in_gate = (dir == Direction::Upstream) ? Gate::Lower : Gate::Upper;
        Gate out_gate = (dir == Direction::Upstream) ? Gate::Upper : Gate::Lower;

        if (lock.level() != target_in_level) {
            lock.pump();
        }

        lock.open(in_gate);
        for (const auto& v : batch) {
            lock.enter(v);
        }
        lock.close(in_gate);

        lock.pump();

        lock.open(out_gate);
        for (const auto& v : batch) {
            lock.exit(v.call_sign);
            report.passed.push_back(v.call_sign);
        }
        lock.close(out_gate);
    }

    report.pump_cycles = lock.pumpCycles();
    return report;
}

}  // namespace kitezh

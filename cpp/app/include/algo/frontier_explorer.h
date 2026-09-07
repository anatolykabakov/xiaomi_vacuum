#pragma once

#include <utility>
#include <vector>

#include "types.h"

namespace project
{
namespace algo
{

struct FrontierExplorerConfig
{
    int min_cluster_cells{5};          // мельче — шум карты, не фронтир
    double min_distance_m{0.30};       // фронтир под роботом целью не считается
    double inflation_radius_m{0.20};   // как у планировщика: цель должна быть достижима диском
    double blacklist_radius_m{0.35};   // «сюда уже не смогли доехать»
};

struct Frontier
{
    double x{0};
    double y{0};
    int cells{0};
    double distance{0};  // от позы робота, м
};

/**
 * Фронтир — свободная ячейка карты, рядом с которой есть неизвестная. Пока фронтиры есть,
 * помещение не исследовано. Кластеры фронтиров сортируются по расстоянию; следующая цель —
 * ближайший кластер, до которого планировщик находит путь по известным свободным ячейкам.
 */
class FrontierExplorer
{
public:
    explicit FrontierExplorer(FrontierExplorerConfig config = {});

    /** Все кластеры фронтиров (центроиды), ближние первыми. */
    std::vector<Frontier> Find(const types::OccupancyMap& map, const types::Pose2D& pose) const;

    /** Ближайший достижимый фронтир, не попавший в чёрный список. false — исследовать нечего. */
    bool NextGoal(
        const types::OccupancyMap& map,
        const types::Pose2D& pose,
        const std::vector<std::pair<double, double>>& blacklist,
        types::Goal& goal
    ) const;

    const FrontierExplorerConfig& config() const { return config_; }

private:
    FrontierExplorerConfig config_;
};

}  // namespace algo
}  // namespace project

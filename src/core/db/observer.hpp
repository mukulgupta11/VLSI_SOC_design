#pragma once

#include "id_types.hpp"
#include <vector>
#include <functional>

namespace soc::db {

// Observer interface for incremental dirty-region and database mutation events
class DatabaseObserver {
public:
    virtual ~DatabaseObserver() = default;

    virtual void on_instance_added(InstanceId /*id*/) {}
    virtual void on_instance_moved(InstanceId /*id*/, Point2D /*old_pos*/, Point2D /*new_pos*/) {}
    virtual void on_net_modified(NetId /*id*/) {}
    virtual void on_dirty_region(const Rect2D& /*bbox*/, LayerId /*layer*/) {}
    virtual void on_database_cleared() {}
};

class DatabaseObserverBus {
public:
    void register_observer(DatabaseObserver* observer) {
        if (observer) {
            m_observers.push_back(observer);
        }
    }

    void unregister_observer(DatabaseObserver* observer) {
        std::erase(m_observers, observer);
    }

    void notify_instance_added(InstanceId id) {
        for (auto* obs : m_observers) obs->on_instance_added(id);
    }

    void notify_instance_moved(InstanceId id, Point2D old_pos, Point2D new_pos) {
        for (auto* obs : m_observers) obs->on_instance_moved(id, old_pos, new_pos);
    }

    void notify_net_modified(NetId id) {
        for (auto* obs : m_observers) obs->on_net_modified(id);
    }

    void notify_dirty_region(const Rect2D& bbox, LayerId layer) {
        for (auto* obs : m_observers) obs->on_dirty_region(bbox, layer);
    }

    void notify_cleared() {
        for (auto* obs : m_observers) obs->on_database_cleared();
    }

private:
    std::vector<DatabaseObserver*> m_observers;
};

} // namespace soc::db

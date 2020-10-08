//
// Created by xinyang on 2020/10/5.
//

#ifndef MYSLAM_OBJ_MANAGER_HPP
#define MYSLAM_OBJ_MANAGER_HPP

#include <unordered_map>
#include <memory>

template<class T>
class ObjManager : public T {
public:
    using Ptr = std::shared_ptr<T>;

    template<class ...Ts>
    static Ptr create(Ts &&...args) {
        Ptr ptr(new ObjManager(factory_id, std::forward<Ts>(args)...));
        obj_set.emplace(factory_id, ptr);
        factory_id++;
        return ptr;
    }

    static const auto &set() {
        return obj_set;
    }

    static Ptr copy(const Ptr &obj) {
        Ptr ptr(new ObjManager(factory_id, *static_cast<T *>(obj.get())));
        obj_set.emplace(factory_id, ptr);
        factory_id++;
        return ptr;
    }

    ~ObjManager() {
        obj_set.erase(_id);
    }

private:
    template<class ...Ts>
    explicit ObjManager(size_t id, Ts &&...args) : _id(id), T(std::forward<Ts>(args)...) {
    }

    static std::unordered_map<size_t, std::weak_ptr<T>> obj_set;
    static size_t factory_id;

    size_t _id;
};

template<class T>
std::unordered_map<size_t, std::weak_ptr<T>> ObjManager<T>::obj_set;

template<class T>
size_t ObjManager<T>::factory_id = 0;

#endif /* MYSLAM_OBJ_MANAGER_HPP */

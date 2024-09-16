#ifndef LRU_CACHE_H
#define LRU_CACHE_H

#include <unordered_map>
#include <list>
#include <stdexcept>

template <typename KeyType, typename ValueType, typename Hash=std::hash<KeyType>, typename Eq=std::equal_to<KeyType>>
class LRUCache {
public:
    typedef std::list<std::pair<KeyType, ValueType>> StorageType;
private:
    size_t capacity_;

    // Storage of key-value pairs, std::list to prevent iterator invalidation
    StorageType cache_items_;  

    // Map into storage type entries for O(1) access
    std::unordered_map<KeyType, typename StorageType::iterator, Hash, Eq> cache_map_;

public:
    // Constructor to initialize cache with a specific capacity
    explicit LRUCache(size_t capacity) : capacity_(capacity) {
        if (capacity_ == 0) {
            throw std::invalid_argument("Capacity must be greater than zero.");
        }
    }

    // Get the value associated with the key
    // Returns true and sets the value if the key is found, otherwise returns false
    bool get(const KeyType& key, ValueType& value) {
        auto it = cache_map_.find(key);
        if (it == cache_map_.end()) {
            return false;  // Key not found
        }
        // Move the accessed item to the front of the cache (most recently used)
        cache_items_.splice(cache_items_.begin(), cache_items_, it->second);
        value = it->second->second;
        return true;
    }

    // Insert or update the key-value pair in the cache
    void insert(const KeyType& key, const ValueType& value) {
        auto it = cache_map_.find(key);
        if (it != cache_map_.end()) {
            // Update existing item and move it to the front
            it->second->second = value;
            cache_items_.splice(cache_items_.begin(), cache_items_, it->second);
        } else {
            // Insert new item
            if (cache_items_.size() == capacity_) {
                // Remove the least recently used item
                auto last = cache_items_.end();
                --last;
                cache_map_.erase(last->first);
                cache_items_.pop_back();
            }
            cache_items_.emplace_front(key, value);
            cache_map_[key] = cache_items_.begin();
        }
    }

    // Remove a key-value pair from the cache
    bool remove(const KeyType& key) {
        auto it = cache_map_.find(key);
        if (it == cache_map_.end()) {
            return false;  // Key not found
        }
        cache_items_.erase(it->second);
        cache_map_.erase(it);
        return true;
    }

    // Get the current size of the cache
    size_t size() const {
        return cache_items_.size();
    }
};

#endif

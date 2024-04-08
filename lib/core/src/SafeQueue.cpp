#include "SafeQueue.hpp"
template class SafeQueue<int16_t>;

template <typename T>
SafeQueue<T>::Container::Container() : next(nullptr) {}

template <typename T>
SafeQueue<T>::QueueManager::QueueManager() : head(nullptr), tail(nullptr) {
    head = new Container();
    tail = head.load();
}

template <typename T>
SafeQueue<T>::QueueManager::~QueueManager() {
    delete head;

    head = tail = nullptr;
}

template <typename T>
void SafeQueue<T>::QueueManager::enq(Container *container) {
        Container *old_tail;
        Container *tail_next;

        container->next = nullptr;
        
        bool inserted = false;
        while (!inserted) {
            old_tail = tail.load(std::memory_order_acquire);
            tail_next = old_tail->next.load(std::memory_order_acquire);

            if (tail_next == nullptr) {
                inserted = old_tail->next.compare_exchange_weak(tail_next, container, std::memory_order_release, std::memory_order_relaxed);
            } else {
                tail.compare_exchange_weak(old_tail, tail_next, std::memory_order_release, std::memory_order_relaxed);
            }
        }
        tail.compare_exchange_weak(old_tail, container, std::memory_order_release, std::memory_order_relaxed);
}

template <typename T>
bool SafeQueue<T>::QueueManager::deq(Container*& container) {
    Container* dummy;
    Container* next;
    int16_t value;

    bool dequeued = false;
    while (!dequeued) {
        dummy = head.load(std::memory_order_acquire);
        next = dummy->next.load(std::memory_order_acquire);
        if (next == nullptr) return false;
        value = next->value;

        dequeued = head.compare_exchange_weak(dummy, next, std::memory_order_release, std::memory_order_relaxed);
    }

    dummy->value = value;
    container = dummy;
    return true;
}

template <typename T>
SafeQueue<T>::SafeQueue(uint32_t size) : queue(), pool() {
    Container *containers = new Container[size];
    for (uint32_t i = 0; i < size; i++) {
        pool.enq(&containers[i]);
    }
}

template <typename T>
SafeQueue<T>::~SafeQueue() {
    Container *c = nullptr;
    while (pool.deq(c)) delete c;
    while (queue.deq(c)) delete c;
}

template <typename T>
bool SafeQueue<T>::enq(T value) {

    Container *c = nullptr;

    if (pool.deq(c)) {
        c->value = value;
        queue.enq(c);
        return true;
    } else {
        return false;
    }
}

template <typename T>
bool SafeQueue<T>::deq(T& value) {
    Container *c = nullptr;

    if (queue.deq(c)) {
        value = c->value;
        pool.enq(c);
        return true;
    } else {
        return false;
    }
}
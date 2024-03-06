#include <Arduino.h>
#include "SafeQueue.hpp"

SafeQueue::Container::Container() : next(nullptr) {}

SafeQueue::QueueManager::QueueManager() : head(nullptr), tail(nullptr) {
    head = new Container();
    tail = head.load();
}

SafeQueue::QueueManager::~QueueManager() {
    delete head;

    head = tail = nullptr;
}

void SafeQueue::QueueManager::enq(Container *container) {
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

bool SafeQueue::QueueManager::deq(Container*& container) {
    Container* dummy;
    Container* next;
    uint16_t value;

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

SafeQueue::SafeQueue(uint32_t size) : queue(), pool() {
    for (uint32_t i = 0; i < size; ++ i) pool.enq(new Container());
}

SafeQueue::~SafeQueue() {
    Container *c = nullptr;
    while (pool.deq(c)) delete c;
    while (queue.deq(c)) delete c;
}

bool SafeQueue::enq(uint16_t value) {

    Container *c = nullptr;

    if (pool.deq(c)) {
        c->value = value;
        queue.enq(c);
        return true;
    } else {
        return false;
    }
}

bool SafeQueue::deq(uint16_t& value) {
    Container *c = nullptr;

    if (queue.deq(c)) {
        value = c->value;
        pool.enq(c);
        return true;
    } else {
        return false;
    }
}
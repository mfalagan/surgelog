#ifndef SAFE_QUEUE_H
#define SAFE_QUEUE_H

#include <stdint.h>
#include <atomic>

// preallocated thread-safe FIFO queue implementation
// TODO: benchmark, test
// TODO: handle allocation failure errors
// TODO: implement dynamic size
template <typename T>
class SafeQueue {
private:
    // container object for storing elements
    struct Container {
        T value;
        std::atomic<Container*> next;

        Container();
    };

    // Michael & Scott queue variation
    // containers are enqueued & dequeued whole
    class QueueManager {
    private:
        std::atomic<Container*> head;
        std::atomic<Container*> tail;

    public:
        QueueManager();
        ~QueueManager();

        void enq(Container*);
        bool deq(Container*&);
    };

    QueueManager queue;
    QueueManager pool;

public:

    SafeQueue(uint32_t size);
    ~SafeQueue();

    // inserts element into queue
    // if full, returns false
    bool enq(T value);
    // retrieves and removes element from queue
    // if empty, returns false, leaving value as-is
    bool deq(T& value);
};

#endif // SAFE_QUEUE_H
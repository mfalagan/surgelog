#ifndef SAFE_QUEUE_H
#define SAFE_QUEUE_H

#include <stdint.h>
#include <atomic>

// TODO: make generic for reusability, benchmark to ensure no performance hit
class SafeQueue {
private:

    struct Container {
        int16_t value;
        std::atomic<Container*> next;

        Container();
    };

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

    bool enq(int16_t value);
    bool deq(int16_t& value);
};

#endif // SAFE_QUEUE_H
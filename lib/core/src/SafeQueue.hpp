#ifndef SAFE_QUEUE_H
#define SAFE_QUEUE_H

#include <stdint.h>
#include <atomic>

// TODO: make generic for reusability, benchmark to ensure no performance hit
class SafeQueue {
private:

    struct Container {
        uint16_t value;
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

        bool is_empty();
    };

    QueueManager queue;
    QueueManager pool;

public:

    SafeQueue(uint32_t size);
    ~SafeQueue();

    bool enq(uint16_t value);
    bool deq(uint16_t& value);

    bool is_full();
    bool is_empty();
};

#endif // SAFE_QUEUE_H
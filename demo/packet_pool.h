#ifndef PACKET_POOL_HH
#define PACKET_POOL_HH

#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <stdbool.h>
#include <unistd.h>
#include <stdint.h>

#define BUFFER_SIZE 1500
#define BUFFER_COUNT 1000
#define MAX_DROPS 3

typedef uint64_t us_t;

typedef struct Packet_QUIC{
    char data[BUFFER_SIZE];
    size_t packet_size;
    us_t recv_time;
    us_t send_time;
    int drop_count;
}Packet_QUIC;

typedef struct Node {
    Packet_QUIC *packet;
    struct Node *rear;
} Node;

typedef struct FIFOQueue {
    Node *front;
    Node *rear;
} FIFOQueue;

void initialize_queue(FIFOQueue *queue);
bool is_queue_empty(FIFOQueue *queue);
void enqueue(FIFOQueue *queue, Packet_QUIC *packet);
Packet_QUIC *dequeue(FIFOQueue *queue);

bool should_drop_packet(float p);

#endif
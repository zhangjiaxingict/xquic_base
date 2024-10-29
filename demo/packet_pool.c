#include "packet_pool.h"

void initialize_queue(FIFOQueue *queue) {
    queue->front = NULL;
    queue->rear = NULL;
}

bool is_queue_empty(FIFOQueue *queue) {
    return queue->front == NULL;
}

void enqueue(FIFOQueue *queue, Packet_QUIC *packet) {
    Node *new_node = (Node *)malloc(sizeof(Node));
    new_node->packet = packet;
    new_node->rear = NULL;
    if (queue->rear == NULL) {
        queue->front = queue->rear = new_node;
    } else {
        queue->rear->rear = new_node;
        queue->rear = new_node;
    }
}

Packet_QUIC *dequeue(FIFOQueue *queue) {
    if (is_queue_empty(queue)) {
        return NULL;
    }
    Node *temp = queue->front;
    Packet_QUIC *packet = temp->packet;
    queue->front = queue->front->rear;
    if (queue->front == NULL) {
        queue->rear = NULL;
    }
    free(temp);
    return packet;
}

bool should_drop_packet(float p) {
    return ((float)rand() / RAND_MAX) < p;
}


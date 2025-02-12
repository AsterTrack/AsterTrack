#ifndef RINGBUFFER_H_
#define RINGBUFFER_H_

#include <string.h>

#include "pdb_msg.h"

#define MSG_QUEUE_SIZE 4

typedef struct
{
	pd_msg buffer[MSG_QUEUE_SIZE];
	size_t begin;
	size_t end;
	bool wrap;
} pdb_msg_queue;


static inline size_t pdb_msg_getOccupied(pdb_msg_queue *queue)
{
	if (queue->end == queue->begin)
		return queue->wrap ? MSG_QUEUE_SIZE : 0;
	else if (queue->end > queue->begin)
		return queue->end - queue->begin;
	else
		return MSG_QUEUE_SIZE + queue->end - queue->begin;
}
static inline void pdb_msg_push(pdb_msg_queue *queue, const pd_msg *data)
{
	memcpy(queue->buffer + queue->end, data, sizeof(pd_msg));
	// If going to wrap, push start along to maintain order
	if (queue->begin == queue->end && queue->wrap)
		queue->begin = (queue->begin + 1) % MSG_QUEUE_SIZE;
	queue->end = (queue->end + 1) % MSG_QUEUE_SIZE;
	if (queue->begin == queue->end)
		queue->wrap = true;
}

// Give null to just drop the data
static inline void pdb_msg_pop(pdb_msg_queue *queue, pd_msg *dest)
{
	if (pdb_msg_getOccupied(queue) == 0)
		return;

	if (dest)
		memcpy(dest, queue->buffer + queue->begin, sizeof(pd_msg));

	queue->begin = (queue->begin + 1) % MSG_QUEUE_SIZE;
	if (queue->wrap && (queue->begin == 0))
		queue->wrap = false;
}

static inline size_t pdb_msg_getFree(pdb_msg_queue *queue)
{
	return MSG_QUEUE_SIZE - pdb_msg_getOccupied(queue);
}

static inline void pdb_msg_flush(pdb_msg_queue *queue)
{
	queue->wrap  = false;
	queue->begin = queue->end = 0;
}

#endif // RINGBUFFER_H

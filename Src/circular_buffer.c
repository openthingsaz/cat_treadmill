/*
 * circular_buffer.c
 *
 *  Created on: 2019. 12. 16.
 *      Author: issacs
 */



#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <assert.h>

#include "circular_buffer.h"
#include "workout.h"

static void advance_pointer(cbuf_handle_t cbuf)
{
	assert(cbuf);

	if(cbuf->full)
    {
        cbuf->tail = (cbuf->tail + 1) % cbuf->max;
    }

	cbuf->head = (cbuf->head + 1) % cbuf->max;

	// We mark full because we will advance tail on the next time around
	cbuf->full = (cbuf->head == cbuf->tail);
}

static void retreat_pointer(cbuf_handle_t cbuf)
{
	assert(cbuf);

	cbuf->full = false;
	cbuf->tail = (cbuf->tail + 1) % cbuf->max;
}

cbuf_handle_t circular_buf_init(exerciseReport* buffer, size_t size)
{
	assert(buffer && size);

	cbuf_handle_t cbuf = malloc(sizeof(circular_buf_t));
	assert(cbuf);

	cbuf->buffer = buffer;
	cbuf->max = size;
	circular_buf_reset(cbuf);

	assert(circular_buf_empty(cbuf));

	return cbuf;
}

void circular_buf_free(cbuf_handle_t cbuf)
{
	assert(cbuf);
	free(cbuf);
}

void circular_buf_reset(cbuf_handle_t cbuf)
{
    assert(cbuf);

    cbuf->head = 0;
    cbuf->tail = 0;
    cbuf->full = false;
}

size_t circular_buf_size(cbuf_handle_t cbuf)
{
	assert(cbuf);

	size_t size = cbuf->max;

	if(!cbuf->full)
	{
		if(cbuf->head >= cbuf->tail)
		{
			size = (cbuf->head - cbuf->tail);
		}
		else
		{
			size = (cbuf->max + cbuf->head - cbuf->tail);
		}

	}

	return size;
}

size_t circular_buf_capacity(cbuf_handle_t cbuf)
{
	assert(cbuf);

	return cbuf->max;
}

void circular_buf_put(cbuf_handle_t cbuf, exerciseReport* data)
{
	assert(cbuf && cbuf->buffer);

    memcpy(&cbuf->buffer[cbuf->head], data, 1);

    advance_pointer(cbuf);
}

int circular_buf_put_non_overwrite(cbuf_handle_t cbuf, exerciseReport* data)
{
    int r = -1;

    assert(cbuf && cbuf->buffer);

    if(!circular_buf_full(cbuf))
    {
        memcpy(&cbuf->buffer[cbuf->head], data, sizeof(exerciseReport));
        advance_pointer(cbuf);
        r = 0;
    }

    return r;
}

int circular_buf_get(cbuf_handle_t cbuf, exerciseReport* data)
{
    assert(cbuf && cbuf->buffer);

    int r = -1;

    if(!circular_buf_empty(cbuf))
    {
        *data = cbuf->buffer[cbuf->tail];
//        memcpy(data, cbuf->buffer[cbuf->head], sizeof(exerciseReport));
        retreat_pointer(cbuf);

        r = 0;
    }

    return r;
}

bool circular_buf_empty(cbuf_handle_t cbuf)
{
	assert(cbuf);

    return (!cbuf->full && (cbuf->head == cbuf->tail));
}

bool circular_buf_full(cbuf_handle_t cbuf)
{
	assert(cbuf);

    return cbuf->full;
}

int circular_buf_get_range(exerciseReport* gbuf, cbuf_handle_t cbuf, uint32_t timeStamp, size_t n)
//int circular_buf_search(cbuf_handle_t cbuf, uint32_t timeStamp)
{
	int i, cnt, index = 0;
	uint32_t search_timeStamp = timeStamp;
	uint32_t max_timeStamp = timeStamp + n;

	assert(gbuf && cbuf && timeStamp && n);

	// data가 있는지 확인
	if(!circular_buf_empty(cbuf))
	{
		return -1;
	}
	else if(cbuf->head > cbuf->tail) //탐색 방법 1
	{
		for(i = cbuf->tail; i < cbuf->head; i++)
		{
			if(cbuf->buffer[i].timeStamp == search_timeStamp)
			{
				//데이터 찾음
				gbuf[index++] = cbuf->buffer[i];
				search_timeStamp += 1;
				if(search_timeStamp >= max_timeStamp) return 0;
				//return (i+1);
			}
		}
	}
	else
	{
		for(i = cbuf->tail; i < cbuf->max; i++)  //탐색 방법 2
		{
			if(cbuf->buffer[i].timeStamp == search_timeStamp)
			{
				gbuf[index++] = cbuf->buffer[i];
				search_timeStamp += 1;
				if(search_timeStamp >= max_timeStamp) return 0;
				//return (i+1);
			}
		}

		cnt = i;

		for(i = 0; i < cbuf->head; i++)  //탐색 방법 3
		{
			if(cbuf->buffer[i].timeStamp == search_timeStamp)
			{
				gbuf[index++] = cbuf->buffer[i+cnt+1];
				search_timeStamp += 1;
				if(search_timeStamp >= max_timeStamp) return 0;
				//return (cnt+i+1);
			}
		}
	}

	return -1;
}

void print_buffer_status(cbuf_handle_t cbuf)
{
	printf("\rFull: %d, empty: %d, size: %d\n",
				 circular_buf_full(cbuf),
				 circular_buf_empty(cbuf),
				 circular_buf_size(cbuf));
}

//#define EXAMPLE_BUFFER_SIZE 10
//void test_circular_buffer( void ) {
//	uint8_t * buffer  = malloc(EXAMPLE_BUFFER_SIZE * sizeof(uint8_t));
//
//	printf("\r\n=== C Circular Buffer Check ===\n");
//
//	cbuf_handle_t cbuf = circular_buf_init(buffer, EXAMPLE_BUFFER_SIZE);
//
//	printf("Buffer initialized. ");
//	print_buffer_status(cbuf);
//
//	printf("\r\n******\nAdding %d values\n", EXAMPLE_BUFFER_SIZE - 1);
//	for(uint8_t i = 0; i < (EXAMPLE_BUFFER_SIZE - 1); i++)
//	{
//		circular_buf_put(cbuf, i);
//		printf("\rAdded %u, Size now: %lu\n", i, circular_buf_size(cbuf));
//	}
//
//	print_buffer_status(cbuf);
//
//	printf("\r\n******\nAdding %d values\n", EXAMPLE_BUFFER_SIZE);
//	for(uint8_t i = 0; i < EXAMPLE_BUFFER_SIZE; i++)
//	{
//		circular_buf_put(cbuf, i);
//		printf("\rAdded %u, Size now: %lu\n", i, circular_buf_size(cbuf));
//	}
//
//	print_buffer_status(cbuf);
//
//	printf("\r\n******\nReading back values: ");
//	while(!circular_buf_empty(cbuf))
//	{
//		uint8_t data;
//		circular_buf_get(cbuf, &data);
//		printf("%u ", data);
//	}
//	printf("\r\n");
//
//	print_buffer_status(cbuf);
//
//	printf("\r\n******\nAdding %d values\n", EXAMPLE_BUFFER_SIZE + 5);
//	for(uint8_t i = 0; i < EXAMPLE_BUFFER_SIZE + 5; i++)
//	{
//		circular_buf_put(cbuf, i);
//		printf("\rAdded %u, Size now: %lu\n", i, circular_buf_size(cbuf));
//	}
//
//	print_buffer_status(cbuf);
//
//	printf("\r\n******\nReading back values: ");
//	while(!circular_buf_empty(cbuf))
//	{
//		uint8_t data;
//		circular_buf_get(cbuf, &data);
//		printf("%u ", data);
//	}
//	printf("\r\n");
//
//	printf("\r\n******\nAdding %d values using non-overwrite version\n", EXAMPLE_BUFFER_SIZE + 5);
//	for(uint8_t i = 0; i < EXAMPLE_BUFFER_SIZE + 5; i++)
//	{
//		circular_buf_put_non_overwrite(cbuf, i);
//	}
//
//	print_buffer_status(cbuf);
//
//	printf("\r\n******\nReading back values: ");
//	while(!circular_buf_empty(cbuf))
//	{
//		uint8_t data;
//		circular_buf_get(cbuf, &data);
//		printf("\r%u ", data);
//	}
//	printf("\r\n");
//
//	free(buffer);
//	circular_buf_free(cbuf);
//}

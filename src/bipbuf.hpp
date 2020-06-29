#pragma once

//
// Author:  Richard Clarke   (@spsrich)
// Date:	2015-05-13
//
// Grateful Acknowledgements: 
//
//       Willem Thiart (https://github.com/willemt/bipbuffer) 
//       Simon Cooke (http://www.codeproject.com/Articles/3479/The-Bip-Buffer-The-Circular-Buffer-with-a-Twist)
//

//

#include <cstring> // memcpy
#include <string>

#define BUPBUF_STATIC 1
#define BIPBUF_SIZE 2048

typedef struct
{
    unsigned int size;

	/* region A */
	unsigned int a_start, a_end;

	/* region B */
	unsigned int b_end;

	/* is B inuse? */
	int b_inuse;

#ifdef BUPBUF_STATIC
    unsigned char data[BIPBUF_SIZE];
#else
    unsigned char data[];
#endif
} bipbuf_t;

/**
* Create a new bip buffer.
*
* malloc()s space
*
* @param[in] size The size of the buffer */
bipbuf_t *bipbuf_new(const unsigned int size);

/**
* Initialise a bip buffer. Use memory provided by user.
*
* No malloc()s are performed.
*
* @param[in] size The size of the array */
void bipbuf_init(bipbuf_t* me, const unsigned int size);

/**
* Free the bip buffer */
void bipbuf_free(bipbuf_t *me);

unsigned char *bipbuf_request(bipbuf_t* me, const unsigned int size);
unsigned int bipbuf_push(bipbuf_t* me, const unsigned int size);

/**
* @param[in] data The data to be offered to the buffer
* @param[in] size The size of the data to be offered
* @return number of bytes offered */
unsigned int bipbuf_offer(bipbuf_t *me, const unsigned char *data, const unsigned int size);

/**
* Look at data. Don't move cursor
*
* @param[in] len The length of the data to be peeked
* @return data on success, NULL if we can't peek at this much data */
unsigned char *bipbuf_peek(const bipbuf_t* me, const unsigned int len);

/**
* Get pointer to data to read. Move the cursor on.
*
* @param[in] len The length of the data to be polled
* @return pointer to data, NULL if we can't poll this much data */
unsigned char *bipbuf_poll(bipbuf_t* me, const unsigned int size);

/**
* @return the size of the bipbuffer */
unsigned int bipbuf_size(const bipbuf_t* me);

/**
* @return 1 if buffer is empty; 0 otherwise */
bool bipbuf_is_empty(const bipbuf_t* me);

/**
* @return how much space we have assigned */
unsigned int bipbuf_used(const bipbuf_t* cb);

/**
* @return bytes of unused space */
unsigned int bipbuf_unused(const bipbuf_t* me);



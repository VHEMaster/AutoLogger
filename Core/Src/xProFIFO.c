/*
 * xProFIFO.c
 *
 *  Created on: Dec 2, 2020
 *      Author: denys.prokhorov
 */

#include "xProFIFO.h"
#include <string.h>

static inline int infoGetSize(sProInfo* info) {
    if(info->write >= info->read) return (info->write - info->read);
    else return (info->capacity - info->read + info->write);
}

static inline uint32_t infoGetAvail(sProInfo* info) { return info->capacity-infoGetSize(info); }
static inline uint32_t infoCorrect(sProInfo* info, uint32_t* param, uint32_t move) { return ((*param + move) % info->capacity); }
static inline void infoMovePar(sProInfo* info, uint32_t* param, uint32_t move) { *param = (uint32_t)infoCorrect(info,param,move); }
static inline uint8_t infoIsSome(sProInfo* info) { return info->read != info->write; }

static inline void protMovePar(sProInfo* info, uint32_t* param, uint32_t move) {
        infoMovePar(info,param,move);
}


inline uint32_t protGetSize(sProFIFO* fifo) {
    uint32_t value;
        value = infoGetSize(&fifo->info);
    return value;
}

inline uint32_t protGetAvail(sProFIFO* fifo) {
    uint32_t value;
        value = infoGetAvail(&fifo->info);
    return value;
}

void protInit(sProFIFO* fifo, void* xBuffer, int xElemSize, int xCapacity) {
    fifo->buffer = xBuffer;
    fifo->info.capacity = xCapacity;
    fifo->info.read = 0;
    fifo->info.write = 0;
    fifo->info.overflow = 0;
    fifo->info.elemsize = xElemSize;
}

void protClear(sProFIFO* fifo) {
        fifo->info.read = 0;
        fifo->info.write = 0;
        fifo->info.overflow = 0;
}

static inline void interPut(sProFIFO* fifo, const void* xData) {
    memcpy((uint8_t*)((uint32_t)fifo->buffer + fifo->info.write * fifo->info.elemsize), (const uint8_t*)xData, fifo->info.elemsize);
}

static inline void interGet(sProFIFO* fifo, void* xData) {
    memcpy((uint8_t*)xData, (uint8_t*)((uint32_t)fifo->buffer + fifo->info.read * fifo->info.elemsize), fifo->info.elemsize);
}

static inline void interLook(sProFIFO* fifo, uint32_t xIndex, void* xData) {
    memcpy((uint8_t*)xData, (uint8_t*)((uint32_t)fifo->buffer + xIndex * fifo->info.elemsize), fifo->info.elemsize);
}

static inline uint32_t interPush(sProFIFO* fifo, const void* xData) {
    uint32_t retval;
    if ((retval = infoGetAvail(&fifo->info))) {
        interPut(fifo,xData);
        infoMovePar(&fifo->info, &fifo->info.write, 1);
    } else { fifo->info.overflow=1; }
    return retval;
}

static inline uint32_t interPull(sProFIFO* fifo, void* xDest) {
    uint32_t retval;
    if ((retval = infoIsSome(&fifo->info))) {
        interGet(fifo,xDest);
        infoMovePar(&fifo->info, &fifo->info.read, 1);
    }
    return retval;
}

static inline uint32_t interPushSequence(sProFIFO* fifo, const void* xData, uint32_t xCount) {
    uint32_t retval = 0; uint32_t i;
    for (i=0; i<xCount; i++) {
        if (!(retval = interPush(fifo, (void*)((uint32_t)xData + i * fifo->info.elemsize)))) { break; }
    }
    return retval;
}

inline uint32_t protPushSequence(sProFIFO* fifo, const void* xData, uint32_t xCount) {
    uint32_t retval;
        retval=interPushSequence(fifo,xData,xCount);
    return retval;
}

inline uint32_t protPush(sProFIFO* fifo, const void* xData) {
    uint32_t retval;
        retval=interPush(fifo,xData);
    return retval;
}

inline uint32_t protPull(sProFIFO* fifo, void* xDest) {
    uint32_t retval;
        retval=interPull(fifo,xDest);
    return retval;
}

inline void protLook(sProFIFO* fifo, uint32_t xOffset, void* xDest) {
        uint32_t aIndex = infoCorrect(&fifo->info, &fifo->info.read, xOffset);
        interLook(fifo,aIndex,xDest);
}

inline void protMoveWrite(sProFIFO* fifo, uint32_t amove) { protMovePar(&fifo->info, &fifo->info.write, amove); }
inline void protMoveRead(sProFIFO* fifo, uint32_t amove) { protMovePar(&fifo->info, &fifo->info.read, amove); }

inline uint8_t protIsSome(sProFIFO* fifo) { return fifo->info.read != fifo->info.write; }

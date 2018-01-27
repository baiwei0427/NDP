// -*- c-basic-offset: 4; tab-width: 8; indent-tabs-mode: t -*-        
#ifndef AEOLUS_QUEUE_H
#define AEOLUS_QUEUE_H

/*
 * Queue for our solution Aeolus 
 */

#define QUEUE_INVALID 0
#define QUEUE_LOW 1
#define QUEUE_HIGH 2


#include <list>
#include "queue.h"
#include "config.h"
#include "eventlist.h"
#include "network.h"
#include "loggertypes.h"

class AeolusQueue : public Queue {
 public:
    AeolusQueue(linkspeed_bps bitrate, mem_b maxsize, EventList &eventlist, QueueLogger* logger);

    virtual void receivePacket(Packet& pkt);
    
    virtual void doNextEvent();
    // should really be private, but loggers want to see
    mem_b _queuesize_low,_queuesize_high;

    int num_headers() const { return _num_headers;}
    int num_packets() const { return _num_packets;}
    int num_stripped() const { return _num_stripped;}
    int num_bounced() const { return _num_bounced;}
    int num_acks() const { return _num_acks;}
    int num_nacks() const { return _num_nacks;}
    int num_pulls() const { return _num_pulls;}
    virtual mem_b queuesize();
    
    virtual void setName(const string& name) 
    {
	   Logged::setName(name); 
	   _nodename += name;
    }

    virtual const string& nodename() 
    { 
        return _nodename; 
    }

    int _num_packets;
    int _num_headers; // only includes data packets stripped to headers, not acks or nacks
    int _num_acks;
    int _num_nacks;
    int _num_pulls;
    int _num_stripped; // count of packets we stripped
    int _num_bounced;  // count of packets we bounced

    int drop_thresh;    // drop threshold for the first-RTT packets

 protected:
    // Mechanism
    void beginService();    // start serving the item at the head of the queue
    void completeService(); // wrap up serving the item at the head of the queue

    int _serv;
    int _ratio_high, _ratio_low, _crt;

    list<Packet*> _enqueued_low;
    list<Packet*> _enqueued_high;
};

#endif

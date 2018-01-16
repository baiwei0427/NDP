#include "config.h"
#include <sstream>
#include <strstream>
#include <iostream>
#include <string.h>
#include <math.h>
#include "network.h"
#include "randomqueue.h"
#include "subflow_control.h"
#include "shortflows.h"
#include "pipe.h"
#include "eventlist.h"
#include "logfile.h"
#include "loggers.h"
#include "clock.h"
#include "ndp.h"
#include "compositequeue.h"
#include "firstfit.h"
#include "topology.h"
#include "connection_matrix.h"
#include "fat_tree_topology.h"
#include <list>
#include "main.h"

#define PERIODIC 0

uint32_t RTT = 1; // this is per link delay in us; identical RTT microseconds = 0.02 ms
#define DEFAULT_NODES 128	// # of nodes by default
#define DEFAULT_QUEUE_SIZE 8	// queue size in packets by default

FirstFit* ff = NULL;
unsigned int subflow_count = 1;

string ntoa(double n);
string itoa(uint64_t n);

#define USE_FIRST_FIT 0
#define FIRST_FIT_INTERVAL 100

EventList eventlist;
Logfile* lg;	// log file


void print_path(std::ofstream &paths, const Route* rt){
    for (unsigned int i = 1; i < rt->size() - 1; i += 2) {
		RandomQueue* q = (RandomQueue*)rt->at(i);
		if (q != NULL) {
	    	paths << q->str() << " ";
	    } else { 
	    	paths << "NULL ";
    	}
    }

    paths << endl;
}

int main(int argc, char **argv) 
{
	Packet::set_packet_size(9000);	// MTU = 9KB
	eventlist.setEndtime(timeFromSec(0.201));	// Simulation stops at 0.201 second
	Clock c(timeFromSec(5 / 100.), eventlist);

	int no_of_conns = DEFAULT_NODES;	// # of NDP connections
	int cwnd = 15;						// NDP initial window in MTU-sized packets
	int no_of_nodes = DEFAULT_NODES;	// # of nodes (servers) in the topology

	mem_b queuesize = memFromPkt(DEFAULT_QUEUE_SIZE);	// per-port buffer size in packets in bytes

	stringstream filename(ios_base::out);
    filename << "logout.dat";	// default name of log file

    RouteStrategy route_strategy = SCATTER_PERMUTE;	// default routing strategy

    // Parse arguments and overide default values
    int i = 1;
    while (i < argc) {
		if (!strcmp(argv[i], "-o")) {	// output file
	    	filename.str(std::string());
	    	filename << argv[i + 1];
	    	i++;
		} else if (!strcmp(argv[i], "-sub")) {	// # of subflows
	    	subflow_count = atoi(argv[i + 1]);
	    	i++;
		} else if (!strcmp(argv[i], "-conns")){	// # of connections
	    	no_of_conns = atoi(argv[i + 1]);
	    	i++;
		} else if (!strcmp(argv[i], "-nodes")){	// # of nodes in the topology
	    	no_of_nodes = atoi(argv[i + 1]);
	    	i++;
		} else if (!strcmp(argv[i], "-cwnd")){	// NDP initial window in packets
	    	cwnd = atoi(argv[i + 1]);
	    	i++;
		} else if (!strcmp(argv[i], "-q")){	// per-port buffer size 
	    	queuesize = memFromPkt(atoi(argv[i+1]));
	    	i++;
		} else if (!strcmp(argv[i],"-strat")){	// routing strategy
	    	if (!strcmp(argv[i + 1], "perm")) {
				route_strategy = SCATTER_PERMUTE;
	    	} else if (!strcmp(argv[i + 1], "rand")) {
				route_strategy = SCATTER_RANDOM;
	    	} else if (!strcmp(argv[i + 1], "pull")) {
				route_strategy = PULL_BASED;
	    	} else if (!strcmp(argv[i + 1], "single")) {
				route_strategy = SINGLE_PATH;
	    	}
	    	i++;
		} else {

		}
		i++;
    }

    // Set seed for random number generator
    srand(13);

    // Print simulation settings
    cout << "Using subflow count " << subflow_count <<endl;
	cout << "conns " << no_of_conns << endl;
    cout << "requested nodes " << no_of_nodes << endl;
    cout << "cwnd " << cwnd << endl;
    cout << "Logging to " << filename.str() << endl;

	//Log file 
    Logfile logfile(filename.str(), eventlist);
    lg = &logfile;	

	int tot_subs = 0;	// total # of subflows
    int cnt_con = 0;	// total # of connections

    // We then start the logfile, set it to record events from simulator time zero.
    logfile.setStartTime(timeFromSec(0));
    // The NdpSinkLoggerSampling object will iterate through all NdpSinks and log their rate every 10ms. 
    // This allows us to get throughput measurements after the experiment finishes.
    NdpSinkLoggerSampling sinkLogger = NdpSinkLoggerSampling(timeFromMs(10), eventlist);
    logfile.addLogger(sinkLogger);
    NdpTrafficLogger traffic_logger = NdpTrafficLogger();
    logfile.addLogger(traffic_logger);
	NdpRtxTimerScanner ndpRtxScanner(timeFromMs(10), eventlist);

    // Build a fat-tree topology
    FatTreeTopology* top = new FatTreeTopology(no_of_nodes, 
    										   queuesize, 
					       				       &logfile, 
					       				       &eventlist,
					       				       ff,			// Unknown parameter
					       				       COMPOSITE,	// Type of queue for NDP
					       				       0);

	no_of_nodes = top->no_of_nodes();
	cout << "actual nodes " << no_of_nodes << endl;

    NdpSrc* ndpSrc;
    NdpSink* ndpSnk;

    Route* routeout, *routein;
    double extrastarttime;

    vector<const Route*>*** net_paths;
    net_paths = new vector<const Route*>**[no_of_nodes];
    int* is_dest = new int[no_of_nodes];
    
    for (int i = 0; i < no_of_nodes; i++){
		is_dest[i] = 0;
		net_paths[i] = new vector<const Route*>*[no_of_nodes];
		for (int j = 0; j < no_of_nodes; j++) {
	    	net_paths[i][j] = NULL;
		}
    }

    // Permutation connections
    ConnectionMatrix* conns = new ConnectionMatrix(no_of_nodes);
    conns->setPermutation(no_of_conns);    
    cout << "Running perm with " << no_of_conns << " connections" << endl;

    // used just to print out stats data at the end
    list <const Route*> routes;
    list <NdpSrc*> ndp_srcs;

	// initialize all sources/sinks
    NdpSrc::setMinRTO(50000);	//increase RTO to 50000us to avoid spurious retransmits
    NdpSrc::setRouteStrategy(route_strategy);
    NdpSink::setRouteStrategy(route_strategy);

    int connID = 0;
    map<int,vector<int>*>::iterator it;

    // for each connection group
	for (it = conns->connections.begin(); it != conns->connections.end(); it++) {
		int src = (*it).first;	// a single source
		vector<int>* destinations = (vector<int>*)(*it).second;	// several connections in this group

		// for each destination
		for (unsigned int dst_id = 0; dst_id < destinations->size(); dst_id++) {
			connID++;
	    	int dest = destinations->at(dst_id);
	    	cout << connID << " (" << src << "->" << dest << ") ";
		}
		cout << endl;
	}

	return 0;
}

string ntoa(double n) {
    stringstream s;
    s << n;
    return s.str();
}

string itoa(uint64_t n) {
    stringstream s;
    s << n;
    return s.str();
}

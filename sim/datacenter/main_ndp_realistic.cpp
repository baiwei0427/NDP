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
#include <fstream>
#include "main.h"

using namespace std;

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


void print_path(std::ofstream &paths, const Route* rt)
{
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
	char *trace_file_name = NULL;
	Packet::set_packet_size(1500);	// MTU = 1.5KB
	eventlist.setEndtime(timeFromSec(2.001));	// Simulation stops at 2.001 second
	Clock c(timeFromSec(2), eventlist);

	int no_of_conns = 0;				// # of NDP connections / flows
	int cwnd = 23;						// NDP initial window in MTU-sized packets
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
		} else if (!strcmp(argv[i], "-conns")){	// # of connections / flows
	    	no_of_conns = atoi(argv[i + 1]);
	    	i++;
		} else if (!strcmp(argv[i], "-nodes")){	// # of nodes in the topology
	    	no_of_nodes = atoi(argv[i + 1]);
	    	i++;
		} else if (!strcmp(argv[i], "-cwnd")){	// NDP initial window in packets
	    	cwnd = atoi(argv[i + 1]);
	    	i++;
		} else if (!strcmp(argv[i], "-q")){	// per-port buffer size 
	    	queuesize = memFromPkt(atoi(argv[i + 1]));
	    	i++;
	    } else if (!strcmp(argv[i], "-trace")) {	// flow trace file
	    	trace_file_name = argv[i + 1];
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

    if (!trace_file_name || no_of_conns == 0) {
    	if (!no_of_conns) {
    		cout << "Number of connections should be specified" << endl;
    	}

    	if (!trace_file_name) {
    		cout << "Trace file should be specified" << endl;
    	}
    	
    	return 0;
    }

    // Set seed for random number generator
    srand(13);

    // Print simulation settings
    cout << "Using subflow count " << subflow_count <<endl;
	cout << "conns " << no_of_conns << endl;
	cout << "trace file " << trace_file_name << endl;
    cout << "requested nodes " << no_of_nodes << endl;
    cout << "cwnd " << cwnd << endl;
    cout << "queue size " << queuesize << endl;
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
    // NdpSinkLoggerSampling sinkLogger = NdpSinkLoggerSampling(timeFromMs(10), eventlist);
    // logfile.addLogger(sinkLogger);
    
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

	// outgoing (src->dst) and incoming (dst->src) routes
    Route* routeout, *routein;

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
    conns->setRandom(no_of_conns);    
    cout << "Running perm with " << no_of_conns << " connections" << endl;

	// initialize all sources/sinks
    NdpSrc::setMinRTO(50000);	//increase RTO to 50000us to avoid spurious retransmits
    NdpSrc::setRouteStrategy(route_strategy);
    NdpSink::setRouteStrategy(route_strategy);

    // used just to print out stats data at the end
    list <const Route*> routes;
    list <NdpSrc*> ndp_srcs;

    // open the flow trace file
	ifstream trace_file(trace_file_name);

    int connID = 0;
    map<int,vector<int>*>::iterator it;

    // for each connection group (a single src to multiple destinations)
	for (it = conns->connections.begin(); it != conns->connections.end(); it++) {
		int src = (*it).first;	// a single source
		vector<int>* destinations = (vector<int>*)(*it).second;	// several connections in this group

		vector<int> subflows_chosen;

		// for each destination 
		for (unsigned int dst_id = 0; dst_id < destinations->size(); dst_id++) {
			connID++;
	    	int dest = destinations->at(dst_id);
	    	cout << connID << " (" << src << "->" << dest << ") ";

	    	// set paths from source to destination
	    	if (!net_paths[src][dest]) {
				vector<const Route*>* paths = top->get_paths(src, dest);
				net_paths[src][dest] = paths;
				// cout << src << "->" << dest << " " << paths->size() << " paths" << endl;

				for (unsigned int i = 0; i < paths->size(); i++) {
		    		routes.push_back((*paths)[i]);
				}
	    	}

	    	// set paths from destination to source 
	    	if (!net_paths[dest][src]) {
				vector<const Route*>* paths = top->get_paths(dest, src);
				net_paths[dest][src] = paths;
	    	}

	    	// for each subflow? I guess
	    	// Now we only have a single subflow
	    	for (int connection = 0; connection < 1; connection++) {
				subflows_chosen.clear();

				int it_sub;
				int crt_subflow_count = subflow_count;

				tot_subs += crt_subflow_count;	// update total # of subflows
				cnt_con++;	// total # of connections

				// it_sub = min(crt_subflow_count, net_paths[src][dest]->size())
				it_sub = crt_subflow_count > net_paths[src][dest]->size() ? net_paths[src][dest]->size() : crt_subflow_count;

				// read a line from the flow trace file
				string line;
				uint64_t flow_size;
				double start_time;
				getline(trace_file, line);
				istringstream iss(line);
				iss >> flow_size >> start_time;
				cout << flow_size << " " << start_time << endl;

				// NDP sender
				NdpSrc* ndpSrc = new NdpSrc(NULL, NULL, eventlist);
				ndpSrc->setCwnd(cwnd * Packet::data_packet_size());
				ndpSrc->set_flowsize(flow_size);
				ndp_srcs.push_back(ndpSrc);

				// NDP receiver
				NdpSink* ndpSnk = new NdpSink(eventlist, 1);	// pull at line rate

				ndpSrc->setName("ndp_" + ntoa(src) + "_" + ntoa(dest)+"("+ntoa(connection)+")");
				logfile.writeName(*ndpSrc);
				ndpSnk->setName("ndp_sink_" + ntoa(src) + "_" + ntoa(dest)+ "("+ntoa(connection)+")");
				logfile.writeName(*ndpSnk);

				ndpRtxScanner.registerNdp(*ndpSrc);

				// Choose a path randomly
				int choice = rand() % net_paths[src][dest]->size();
				subflows_chosen.push_back(choice);

				routeout = new Route(*(net_paths[src][dest]->at(choice)));
				routeout->push_back(ndpSnk);
	  			routein = new Route(*top->get_paths(dest,src)->at(choice));
				routein->push_back(ndpSrc);

	  			ndpSrc->connect(*routeout, *routein, *ndpSnk, timeFromSec(start_time));

	  			// I don't understand this part
	  			switch(route_strategy) {
					case SCATTER_PERMUTE:
					case SCATTER_RANDOM:
					case PULL_BASED: {
		    			ndpSrc->set_paths(net_paths[src][dest]);
		    			ndpSnk->set_paths(net_paths[dest][src]);

		    			vector<const Route*>* rts = net_paths[src][dest];
		    			const Route* rt = rts->at(0);
		    			PacketSink* first_queue = rt->at(0);
		    			if (ndpSrc->_log_me) {
							cout << "First hop: " << first_queue->nodename() << endl;
							QueueLoggerSimple queue_logger = QueueLoggerSimple();
							logfile.addLogger(queue_logger);
							((Queue*)first_queue)->setLogger(&queue_logger);
		    
							ndpSrc->set_traffic_logger(&traffic_logger);
		    			}
		    			break;
					}
					default:
		    			break;
				}

				//sinkLogger.monitorSink(ndpSnk);
	    	}
		}
	}

	// close the flow trace file
	trace_file.close();

    cout << "Mean number of subflows " << ntoa((double)tot_subs/cnt_con)<<endl;
    cout << "Loaded " << connID << " connections in total" << endl;

    // Record the setup
    int pktsize = Packet::data_packet_size();
    logfile.write("# pktsize=" + ntoa(pktsize) + " bytes");
    logfile.write("# subflows=" + ntoa(subflow_count));
    logfile.write("# hostnicrate = " + ntoa(HOST_NIC) + " pkt/sec");
    logfile.write("# corelinkrate = " + ntoa(HOST_NIC*CORE_TO_HOST) + " pkt/sec");
    //logfile.write("# buffer = " + ntoa((double) (queues_na_ni[0][1]->_maxsize) / ((double) pktsize)) + " pkt");
    double rtt = timeAsSec(timeFromUs(RTT));
    logfile.write("# rtt =" + ntoa(rtt));

    // GO!
    while (eventlist.doNextEvent()) {
    	
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

/**
 *  \file   real time_mobility.c
 *  \brief  real time routing over ad hoc networks
 *  \author jm
 *  \date   2019
 **/
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <sys/time.h>
#include <string.h>
#include <entity.h>



/* ************************************************** */
/* ************************************************** */
model_t model =  {
    "real time routing over networks",
    "liu",
    "0.1",
    MODELTYPE_ROUTING, 
    {NULL, 0}
};


/* ************************************************** */
/* ************************************************** */
#define HELLO_PACKET 0
#define DATA_PACKET  1
#define REPLAY_PACKET 2

#define MAX_QUEUE 10
#define MAX_DEADLINE 50000000
#define MIN_DEADLINE 10000000
#define NB_timeout 500000000

#define PI 3.14159

#define ENERGY 10000000000
#define ENERGY2 1000
#define T_b 1000
#define SINK 24
#define SINK_X 500
#define SINK_Y 500
#define SINK_Z 0
#define RANGE 180
#define NUMBER 25
#define M 3.1415
#define DATA_SIZE 1016
#define MIN_LINK 0.3
#define MAX_LINK 0.8

#define N 5 //the size of windows
#define B 0.5
#define W 0.8//the rate of delay in return 
#define M 5000
#define A 0.3
#define HPERIOD 100000000
#define num_fault 14 //the number of failed node
#define Learning_time 5000000000
#define test_time 20000000000
#define epl 0.1
#define change_min_speed 10
#define change_max_speed 20
#define NOT_ACK 3
#define endsendtime 10000000000


typedef int entityid_t;

/* ************************************************** */
/* ************************************************** */
struct hello_p{
	int type;
	nodeid_t src;
	position_t src_pos;
	double energy;
	double r;
	double max_q;
	double speed;
	angle_t angle;
	int ilde;
};

struct replay_p{
	int type;
	nodeid_t res;
	position_t res_pos;
	double r;
	double max_q;
};

struct data_p{
	int type;
	nodeid_t src;
	position_t src_pos;
	nodeid_t dst;
    	position_t dst_pos;
	uint64_t deadline;
	struct timeval start_time;
	uint64_t start_deadline;
	int hop;
	uint64_t start_t;
	uint64_t routt;
};

struct routing_header {
    nodeid_t dst;
    position_t dst_pos;
    nodeid_t src;
    position_t src_pos;
    int type;
};

struct neighbor {
	uint32_t *windows;
	uint32_t *wait;
    	int id;
    	position_t position;
    	uint64_t time;
    	double q;
    	double r;
    	double a;
	double max_q;
	double link;
	double error_location;
	double prospeed;
	double energy;
	double speed;
	int window_size;
	int wait_size;
	angle_t angle;
	int hello_number;

	double lq;

	/*calcute delay*/
	uint32_t delay;
	uint32_t macd;
	uint32_t wait_time;
	uint32_t min_delay;
	uint32_t max_delay;

	int cid;//cani id

	int choose_num;// choosed number

	int ilde;
						
};


struct queue_ele{
	packet_t *pack;
	uint64_t come;
	uint64_t leave;
};

struct data_send{
	uint64_t send;
	int seq;
};

struct nodedata {
    	void *neighbors;
	void * data_st;
	int *packet_arriveid;
    	int overhead;
	double q;
	double max_q;
	double r;
	double energy_node;
	int max_hop;
	int avg_hop;

    	uint64_t hello_start;
    	uint64_t hello_period;
    	uint64_t nb_timeout;
    	uint64_t data_start;
    	uint64_t data_period;
	uint64_t start_hello;
	uint64_t nb_maxdelay;
	double max_delay;
	double avg_delay;
	uint64_t max_d;
	uint64_t avg_d;
   	// double deadline;
	uint64_t r_update;
    	//int hop; 
	uint64_t hello_time;//ETX,rycle  
        uint64_t start_send;    

    	/* stats */
	int length_qu;
    	int old_nb;
    	int add_nb;
	int del_nb;
    	int data_noroute;
  	int tx_pack;
 	int rx_pack;
	int rrxp;
  	int meet_time;
	int total_hello;
	int hop;
	int count;
	int old_pack;

	int ilde;

        int after_learning_tx;
        int after_learning_rx;
        int test_rx;
        int test_tx;
        double learning_hop;
        double unlearning_hop;
        double test_hop;

        int change_speed_flag;

};

struct data_d_header{
	int type;
};

//uint64_t max_delay=0;
int count = 0;
int tx_pack = 0;
int rx_pack = 0;
int rrxp = 0;
int sp;
int source_send = 0;
//int srcid=rand()%(NUMBER-1);
double avg_enery = 0;

int failed_node[num_fault];
int current_num = 0;

/*while(current_num<num_fault){
    int bl = 1;
    while(bl==1){
    int srcid=rand()%(NUMBER-1);
    //printf("srcid %d\n",srcid);
    for(int i=0;i<current_num;i++){
       if(srcid==failed_num[i]||srcid==0||srcid==SINK){
             break;
       }
    }
   failed_node[current_num] = srcid;
   current_num++;
   bl = 0;
   }
   printf("failed_node %d\n",failed_node[current_num]-1);
}*/


/* ************************************************** */
/* ************************************************** */
int hello_callback(call_t *c, void *args);
int data_callback(call_t *c, void *args);
int r_callback(call_t *c, void *args);
void rx_hello(call_t *c, packet_t *packet);
void rx_data(call_t *c, packet_t *packet);
void rx_replay(call_t *c, packet_t *packet);
void rx_ack(call_t *c, packet_t *packet);
//void update_Q(call_t *c, packet_t *packet,int type,uint64_t backtrack);
void update_Q(call_t *c, int from, uint64_t delay);
void update_wait(call_t *c, int from, uint64_t queue_delay);
struct neighbor * get_nexthop(call_t *c, position_t *dst,packet_t *packet);
void add_neighbor(call_t *c, struct hello_p *header);
int neighbor_timeout(void *data, void *arg);
void forward(call_t *c,packet_t *packet,uint64_t come_time);
double locate_error(call_t *c,struct neighbor *nb);
void init_q(call_t *c,struct neighbor *nb);
double neighbor_relation(call_t *c,struct neighbor *nb);
void reforward(call_t *c,packet_t *packet);
int statictics_packet(call_t *c, packet_t *packet);
double get_neighbor_lq(struct neighbor *nb,call_t *c);



/* ************************************************** */
/* ************************************************** */
int init(call_t *c, void *params) {
    return 0;
}

int destroy(call_t *c) {
    return 0;
}


/* ************************************************** */
/* ************************************************** */
int setnode(call_t *c, void *params) {
    struct nodedata *nodedata = malloc(sizeof(struct nodedata));
    param_t *param;
   
    /* default values */
	if(c->node==SINK)
		nodedata->packet_arriveid = (int *)malloc(sizeof(int)*M);
	else 
		nodedata->packet_arriveid = NULL;

    	nodedata->neighbors = das_create();
	nodedata->data_st = das_create();
	nodedata->length_qu = 0;
	nodedata->hello_start = 0;
	nodedata->hello_period = 100000000;
    	nodedata->nb_timeout = 300000000;
    	nodedata->data_start = 500000000;
    	nodedata->data_period = 50000000;
    	nodedata->r_update = 100000000;
    	nodedata->data_noroute = 0;
    	nodedata->tx_pack = 0;
    	nodedata->rx_pack = 0;
	nodedata->rrxp = 0;
        nodedata->after_learning_tx = 0;
        nodedata->after_learning_rx = 0;
    	nodedata->meet_time = 0;
	nodedata->max_q = 0;
	nodedata->q = 0;
	nodedata->r = 0;
	nodedata->add_nb = 0;
	nodedata->del_nb = 0;
	nodedata->old_nb = 0;
	nodedata->start_hello = 0;
	nodedata->total_hello = 0;
	nodedata->energy_node = ENERGY;
	nodedata->hop = 0;
	nodedata->count = 0;
        nodedata->learning_hop = 0;
        nodedata->unlearning_hop = 0;
	nodedata->max_delay = 0;
	nodedata->max_hop = 0;
	nodedata->avg_delay = 0;
	nodedata->avg_hop = 0;
	nodedata->old_pack = -1;
	nodedata->nb_maxdelay = 0;
	nodedata->hello_time = 0;
	nodedata->ilde = 0;
        nodedata->test_rx = 0;
        nodedata->test_tx = 0;
        nodedata->test_hop = 0;
        nodedata->change_speed_flag = 0;
        //printf("nnb_timeout %d\n",nodedata->nb_timeout);
	int temp = (NUMBER-2)/num_fault;
        //printf("temp %d\n",temp);
        
        int i = 0;
	/*if ((c->node!=0)&&(c->node!=SINK)&&(c->node % temp==0)&&(current_num<num_fault)){
                current_num ++;
                printf("current_num %d\n",current_num);
                entity_t *entity1 = get_entity_by_name("liner");
		int c_id = entity1->id;
		call_t c1={c_id,c->node,-1};
		entity1->methods->energy.set_energy(&c1,ENERGY2);
		nodedata->energy_node = ENERGY2;
	}*/
        /*for(int i=0;i<num_fault;i++){
            if(c->node==failed_node[i]){
                entity_t *entity1 = get_entity_by_name("liner");
		int c_id = entity1->id;
		call_t c1={c_id,c->node,-1};
		entity1->methods->energy.set_energy(&c1,ENERGY2);
		nodedata->energy_node = ENERGY2;
            }
        }
	FILE *ft = fopen("timeout.txt","ab");

    	/* get params */
    	das_init_traverse(params);
       // printf("%s\n","1");
    	while ((param = (param_t *) das_traverse(params)) != NULL) {
        //printf("%s\n","2");
        if (!strcmp(param->key, "data_start")) {
            if (get_param_time(param->value, &(nodedata->data_start))) {
                goto error;
            }
        }
       // printf("%s\n","3");
        if (!strcmp(param->key, "data_period")) {
            if (get_param_time(param->value, &(nodedata->data_period))) {
                goto error;
            }
        }
		if (!strcmp(param->key, "hello_start")) {
            if (get_param_time(param->value, &(nodedata->hello_start))) {
                goto error;
            }
        }
        if (!strcmp(param->key, "hello_period")) {
           // printf("%s\n","33");
            //printf("hello_period %d\n",nodedata->hello_period);     
            if (get_param_time(param->value, &(nodedata->hello_period))) {
                goto error;
            }
        }
        //printf("%s\n","4");
	if (!strcmp(param->key, "nb_timeout")) {
                    //printf("%s\n","5");
                    //printf("nb_timeout %d\n",nodedata->nb_timeout);
            if (get_param_time(param->value, &(nodedata->nb_timeout))) {
                //printf("%s\n","11111111111111111");
                //fprintf(ft,"nb_timeout %d\n",nodedata->nb_timeout);
                //fclose(ft);
                
                goto error;
            }
        }
        if (!strcmp(param->key, "r_update")) {
            //printf("%s\n","66");
            if (get_param_time(param->value, &(nodedata->r_update))) {
                goto error;
            }
        }
	
    }
	set_node_private_data(c, nodedata);
    	//FILE *fp;
	//fp = fopen("next.txt","ab");
        printf("set_time %u\n",get_time());
	printf("[SETNODE] Node %d , position_x = %f ,position_y = %f,position_z = %f energy %f\n",c->node,get_node_position(c->node)->x,get_node_position(c->node)->y,get_node_position(c->node)->z,nodedata->energy_node);
	//fclose(fp);
        //printf("%s\n","111111");
	/*FILE *p;
	p = fopen("enery.txt","ab");
	fprintf(p,"node %d in_enery %f\n",c->node,nodedata->energy_node);
	fclose(p);*/
    	return 0;
    
 error:
    free(nodedata);
    return -1;
}

int unsetnode(call_t *c) {
    	struct nodedata *nodedata = get_node_private_data(c);
    	struct neighbor *neighbor = NULL;
	struct data_send *datasend = NULL;
	//FILE *fpt;
        //fpt = fopen("print.txt","ab");
	//printf("[NEIGHBOR] Node %d ",c->node);
	/*if(c->node==0)
		printf("node 0 rx %d\n",nodedata->rx_pack);
	if(c->node==29)
		printf("node 0 tx %d\n",nodedata->tx_pack);*/
	
    	while ((neighbor = (struct neighbor *) das_pop(nodedata->neighbors)) != NULL) {
		//printf("%d ",neighbor->id);
              free(neighbor);
    	}
	das_destroy(nodedata->neighbors); 
	//printf("%s \n","neighbor end");

	while ((datasend = (struct data_send *) das_pop(nodedata->data_st)) != NULL) {
              free(datasend);
    	}
	das_destroy(nodedata->data_st); 

    /* we print node stats before exit */
	rx_pack += nodedata->rx_pack;
	tx_pack += nodedata->tx_pack;
	rrxp += nodedata->rrxp;
	
	avg_enery += (ENERGY-nodedata->energy_node);
	/*FILE *fp;
	fp = fopen("enery.txt","ab");
	fprintf(fp,"node %d enery %f\n",c->node,nodedata->energy_node);
	fclose(fp);*/
	if(c->node==SINK){
		FILE *maxf,*avgf;
		maxf = fopen("max_delay.txt","ab");
		avgf = fopen("avg_delay.txt","ab");
		if(count!=0){
		nodedata->avg_delay = nodedata->avg_delay/count;
		//nodedata->avg_d = nodedata->avg_d/count;
		nodedata->avg_hop  = nodedata->avg_hop/count;
		}
		double max_delay = nodedata->max_delay/1000000.0;
                double s = count/1000;
                if(s==0)
                    s = 0.0001;
		if(max_delay>0){
                    fprintf(maxf,"max_delay %f\n", max_delay);
                }
		double delay_s = nodedata->avg_delay/1000000.0;
		sp = nodedata->rx_pack;
		fprintf(avgf,"avg_delay %f avg_hop %d %d ",delay_s,nodedata->avg_hop,count);
                //printf("count %d\n",count);
		fclose(maxf);
		fclose(avgf);
		/*FILE *meetf = fopen("meettime.txt","ab");
		fprintf(meetf,"meet %d\n",nodedata->meet_time);
		fclose(meetf);*/
		//fprintf(fp,"avg_d %u  ",nodedata->avg_d/count);
		//fprintf(fp,"max_d %u\n",nodedata->max_d);
	}
	//fprintf(fp,"node %d total_rx %d total_tx %d total_meet_time %d\n",c->node,nodedata->rx_pack,nodedata->tx_pack,nodedata->meet_time);
	//fclose(fp);
	FILE *f;
	//f = fopen("td_v_100.txt","ab");
	if(c->node==0){
		double t = (1.0*tx_pack/NUMBER);
		double r = (1.0*rx_pack/NUMBER);
		double rr = (1.0*rrxp/NUMBER);
                double p =((tx_pack+rrxp)/sp)/NUMBER;
		//fprintf(f,"power %d sp %d\n",tx_pack+rrxp,sp);
		FILE *txf;
		//maxf = fopen("mt2dinteral10_addr.txt","ab");
		txf = fopen("tx_packet.txt","ab");
		fprintf(txf,"%d\n",nodedata->tx_pack);
		fclose(txf);
		/*FILE *fp;
		fp = fopen("total_enery_mono2_10.txt","ab");
		avg_enery = avg_enery/NUMBER;
		fprintf(fp,"enery %f\n",avg_enery);
		fclose(fp);*/
                printf("tx_packet %d nodedata->after_learning_tx %d\n", nodedata->tx_pack, nodedata->after_learning_tx);
	}
	//fclose(f);
        if(c->node == SINK){
            printf("rx_packet %d nodedata->after_learning_rx %d\n", nodedata->rx_pack, nodedata->after_learning_rx);
            printf("learning_avg_hop %f unlr_avg_hop %f\n", nodedata->learning_hop/(nodedata->rx_pack-nodedata->after_learning_rx), nodedata->test_hop/nodedata->after_learning_rx);
        }
        printf("unset_time %u\n",get_time());
    	printf("[UNSETNODE] Node %d => total rx = %d , total tx = %d , total meet_time = %d noroute = %d , total_hello = %d \n", c->node, nodedata->rx_pack, nodedata->tx_pack,nodedata->meet_time,nodedata->data_noroute,nodedata->total_hello);
	printf("[UNSETNODE] Node %d , position_x = %f ,position_y = %f,position_z = %f \n",c->node,get_node_position(c->node)->x,get_node_position(c->node)->y,get_node_position(c->node)->z);
	FILE *pt;
	pt = fopen("end_pos.txt","ab");
	fprintf(pt,"[UNSETNODE] Node %d , position_x = %f ,position_y = %f,position_z = %f \n",c->node,get_node_position(c->node)->x,get_node_position(c->node)->y,get_node_position(c->node)->z);
	fclose(pt);
   // fclose(fpt);
    free(nodedata);
    return 0;
}


/* ************************************************** */
/* ************************************************** */
int bootstrap(call_t *c) {
    printf("node %d qmr_set\n", c->node);
    struct nodedata *nodedata = get_node_private_data(c);
    call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
    
    /* get mac header overhead */
    nodedata->overhead = GET_HEADER_SIZE(&c0);
        
    /* scheduler first hello */
    if (nodedata->hello_period >0) {
        scheduler_add_callback(get_time()+nodedata->hello_start+get_random_double()*nodedata->hello_period, c, hello_callback, NULL);
    }

    /* scheduler data packet */
    if (c->node == 0) {
        nodedata->start_send = get_time();
        scheduler_add_callback(get_time()+nodedata->data_start, c, data_callback, NULL);
    }
	 scheduler_add_callback(get_time()+nodedata->r_update, c, r_callback, NULL);
	
    return 0;
}

int ioctl(call_t *c, int option, void *in, void **out) {
    return 0;
}


/* Periodic exchange of hello packets */
int hello_callback(call_t *c, void *args) {
    //printf("[HELLO]  Node %d time %u\n",c->node,get_time());
    struct nodedata *nodedata = get_node_private_data(c);
    struct neighbor *neighbor = NULL;
    call_t c1 = {9,SINK,c->from};
    position_t *dst = get_node_position(SINK);
    position_t *sour = get_node_position(c->node);
    double d = distance(sour, dst);
    int bl = 0;
    int m = das_getsize((void *)nodedata->neighbors);
    das_init_traverse(nodedata->neighbors);
   
    nodedata->start_hello = get_time();
    nodedata->total_hello++;

    if(get_time()-nodedata->hello_time>3*nodedata->hello_period)
	nodedata->hello_time = get_time();

    call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
    destination_t destination = {BROADCAST_ADDR, {-1, -1, -1}};
    packet_t *packet = packet_create(c, nodedata->overhead + sizeof(struct hello_p), -1);
    struct hello_p *hello = (struct hello_p *) (packet->data + nodedata->overhead);
    position_t *pos = get_node_position(c->node);
	
    /* set mac header */
    if (SET_HEADER(&c0, packet, &destination) == -1) {
        packet_dealloc(packet);
        return -1;
    }
	double nb_speed;
	angle_t nb_angle;
	if(c->node==SINK||c->node==0){
		nb_angle.xy = 0;
		nb_angle.z = 0;
		nb_speed = 0;
	}
	else{
    	entity_t *entity_m = get_entity_by_name("torus central");
	int m_id = entity_m->id;
	call_t cm={m_id,c->node,-1};
	//struct nodedata *nodedatam = get_node_private_data(&cm);
	nb_speed = entity_m->methods->mobility.get_speed(&cm);
	nb_angle = entity_m->methods->mobility.get_angle(&cm);
	}
    
    /* set header */
    	hello->type       = HELLO_PACKET;
    	hello->src        = c->node;
    	hello->src_pos.x = pos->x;
    	hello->src_pos.y = pos->y;
    	hello->src_pos.z = pos->z;
	hello->energy = nodedata->energy_node;
	hello->max_q = nodedata->max_q;
	hello->r     = nodedata->r;
	hello->speed = nb_speed;
	hello->angle = nb_angle;
	hello->ilde = 0;
   	TX(&c0, packet);	 

	//FILE *fe = fopen("energy.txt","ab");
	//fprintf(fe,"be_energy %f\n",nodedata->energy_node);
	entity_t *entity_e = get_entity_by_name("liner");
	int ce_id = entity_e->id;
	call_t ce={ce_id,c->node,-1};
	uint64_t duration = T_b*packet->size;
	entity_e->methods->energy.consume_tx(&ce,duration,0);
	nodedata->energy_node = entity_e->methods->energy.energy_remaining(&ce);
	//fprintf(fe,"af_energy %f\n",nodedata->energy_node);
	//fclose(fe);
    
	/* check neighbors timeout */
    if (nodedata->nb_timeout > 0) {
        das_selective_delete(nodedata->neighbors, neighbor_timeout, (void *) c);
    }
    
    /* schedules hello */
    scheduler_add_callback(get_time() + nodedata->hello_period, c, hello_callback, NULL);
    //printf("%s \n","end_helloback");
    return 0;
}

int data_callback(call_t *c, void *args) {
    	struct nodedata *nodedata = get_node_private_data(c);
	nodedata->count++;

    	/* send a new data packet */
    	call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
    	//packet_t *packet = packet_alloc(c, nodedata->overhead + sizeof(struct data_p));
	packet_t *packet = packet_create(c, nodedata->overhead + sizeof(struct data_p), DATA_SIZE);
        //printf("Learning_time %ld nodedata->data_period %ld\n", Learning_time, nodedata->data_period);
        packet->learning_flag = 0;
        //printf("learning packet %d\n", Learning_time / nodedata->data_period);
        if(nodedata->tx_pack > Learning_time / nodedata->data_period){
             packet->learning_flag = 1; 
             nodedata->after_learning_tx += 1;
        }
        //printf("source sends pk_id %d\n",packet->id);
    	struct data_p *data_header = (struct data_p *) (packet->data + nodedata->overhead);
	data_header->deadline = get_random_integer_range(MIN_DEADLINE,MAX_DEADLINE);
	//FILE *dt;
	//dt = fopen("deadline.txt","ab");
	//fprintf(dt,"[DATA]  deadline %d\n",data_header->deadline);
	
	data_header->start_deadline = data_header->deadline;
	struct timeval start;
	gettimeofday(&start,0);
	
	//data_header->start_time = get_time();
	//data_header->start_time.tv_usec = start.tv_usec;
	//data_header->start_time.tv_sec = start.tv_sec;
	data_header->start_t = get_time();

	data_header->routt = 0;
	
	//fprintf(fpt,"pack_id = %d , pack_deadline %u\n",packet->id,data_header->deadline);
	call_t cs = {9, SINK, c->from};
	data_header->dst = SINK;
	data_header->dst_pos.x = get_node_position(cs.node)->x;
	data_header->dst_pos.y = get_node_position(cs.node)->y;
	data_header->dst_pos.z = 0;
	data_header->src = 0;
	position_t *position = get_node_position(c->node);
	data_header->src_pos.x = position->x;
	data_header->src_pos.y = position->y;
	data_header->src_pos.z = position->z;
	data_header->type = DATA_PACKET;
	data_header->hop  = 1;
	nodedata->hop++;

	forward(c,packet,get_time());
    
    /* for stats */
       //nodedata->tx_pack++;
       //printf("source trans a packet %d\n", nodedata->tx_pack);

    /* schedule a new callback after actualtime+period */
        //if(get_time())
        if((get_time() - nodedata->start_send) < endsendtime)
             scheduler_add_callback(get_time() + nodedata->data_period, c, data_callback, NULL);

        //printf("%s\n","end_databack");
        return 0;
}

int r_callback(call_t *c, void *args){
	//printf("[RCALL] current %u\n",get_time());
	struct nodedata *nodedata = get_node_private_data(c);
	struct neighbor *neighbor = NULL;
	struct data_send * data_se = NULL;
	double max_q = 0;

	if((nodedata->old_nb+nodedata->add_nb)==0)
		nodedata->r = 0;
	else{
		nodedata->r = 1-((nodedata->add_nb+nodedata->del_nb)*1.0/(nodedata->old_nb+nodedata->add_nb)*1.0);
	}
	
	nodedata->old_nb = nodedata->old_nb+nodedata->add_nb-nodedata->del_nb;
	nodedata->add_nb = 0;
	nodedata->del_nb = 0;
	//printf("Node %d nb_size %d add_nb %d del_nb %d r %f \n",c->node,das_getsize(nodedata->neighbors),nodedata->old_nb,nodedata->del_nb,nodedata->r);

	/* update max_q */
	
	position_t dst;
	dst.x = SINK_X;
	dst.y = SINK_Y;
	dst.z = SINK_Z;
	double d_old = distance(get_node_position(c->node),&dst);
	double d_nb;
	 
	das_init_traverse(nodedata->neighbors);      
    	while ((neighbor = (struct neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
	d_nb = distance(&(neighbor->position),&dst);
        if(neighbor->q>max_q)
		max_q = neighbor->q;
    	} 
	nodedata->max_q = max_q;

	/* penalty without ack */
	uint64_t current = get_time();
	//int bl = 0;
	das_init_traverse(nodedata->data_st); 	 
	while ((data_se = (struct data_send *) das_traverse(nodedata->data_st)) != NULL) {
		//bl = 0;
		//if (bl==0) {
		das_init_traverse(nodedata->neighbors);
		while(((neighbor = (struct neighbor *) das_traverse(nodedata->neighbors)) != NULL)){
			if(neighbor->id==data_se->seq){
				if((current-data_se->send)>=1.2*neighbor->delay){
					neighbor->q = (1-neighbor->a)*neighbor->q+neighbor->a*(-1+(neighbor->r)*neighbor->max_q);
					
					das_delete(nodedata->data_st, data_se);
	
				}
					//bl = 1;
				break;
			}
		}
		//}
	}
	//fclose(fpt);

	/* schedules r */
    scheduler_add_callback(get_time() + nodedata->r_update, c, r_callback, NULL);
	//printf("%s \n","end_rback");
	return 0;
}

double neighbor_relation(call_t *c,struct neighbor *nb){
	uint64_t current = get_time();
	//double d_mobility = (current-nb->time)*nb->speed*1.0/1000000000;
	//double d_old = distance(get_node_position(c->node),&(nb->position));
	double rela_rate;
	 
	position_t pos;
	//3d
	/*pos.x = nb->position.x+((nb->speed * cos(nb->angle.xy)*cos(nb->angle.z))*(current-nb->time+nb->macd))/1000000000;
	pos.y = nb->position.y+((nb->speed * sin(nb->angle.xy)*cos(nb->angle.z))*(current-nb->time+nb->macd))/1000000000;
	pos.z = nb->position.z+((nb->speed * sin(nb->angle.z))*(current-nb->time+nb->macd))/1000000000;
	if(pos.z<0)
		pos.z = 0;*/
	//2d
	pos.x = nb->position.x+((nb->speed * cos(nb->angle.xy))*(current-nb->time+nb->macd))/1000000000;
	pos.y = nb->position.y+((nb->speed * sin(nb->angle.xy))*(current-nb->time+nb->macd))/1000000000;
	pos.z = 0;
	double d_new = distance(get_node_position(c->node),&(pos));
	/*position_t *new_pos = get_node_position(nb->id);
	FILE *fl = fopen("pos.txt","ab");
	fprintf(fl,"node %d x %f y %f z %f\n",nb->id,new_pos->x,new_pos->y,new_pos->z);
	fprintf(fl,"old_x %f old_y %f old_z %f x %f y %f z %f \n",nb->position.x,nb->position.y,nb->position.z,pos.x,pos.y,pos.z);
	fclose(fl);*/
	if(d_new<=RANGE)
		//rela_rate = 1;
		rela_rate = 1-d_new/RANGE;
	else
		rela_rate = 0;
	/*if(d_old+d_mobility<=RANGE){
		rela_rate = 1;
		
	}
	else if(d_old-d_mobility>RANGE)
		rela_rate = 0;
	else{
		double cos_ang = (d_old*d_old+RANGE*RANGE-d_mobility*d_mobility)/(2*d_old*RANGE);
		double angle = 2*acos(cos_ang);
		double L = angle*RANGE;
		double t_area = 0.5*L*d_mobility;
		cos_ang = (d_old*d_old+d_mobility*d_mobility-RANGE*RANGE)/(2*d_old*d_mobility);
		angle = 2*(PI - acos(cos_ang));
		double s_area = 0.5*angle*d_mobility*d_mobility;
		double area = PI*d_mobility*d_mobility-(s_area-t_area);
		rela_rate = area/(PI*d_mobility*d_mobility);
	}*/
	return rela_rate;
}

double get_neighbor_lq(struct neighbor *nb,call_t *c){
	//FILE *fl = fopen("lq.txt","ab");
	//fprintf(fl,"c.id %d nb.id %d\n",c->node,nb->id);
	
	/*uint64_t current = get_time();
	if(current-nb->time>2*HPERIOD)
		nb->lq=nb->lq*A;
	else
		nb->lq=nb->lq*A+(1-A)*1;*/

	 
	double dr = (nb->hello_number*1.0)/3.0;
	//fprintf(fl,"nb %d hello_number %d dr %f\n",nb->id,nb->hello_number,dr);
	/*call_t c1 = {c->entity,nb->id, c->from};
	// c1;
	if(nb->id==0){
		c1.entity = 7;
	}
	else if(nb->id==SINK){
		c1.entity = 9;
	}
	else
		c1.entity = 8;
	//fprintf(fl,"node %d c1.entity %d c1.from %d\n",nb->id,c1.entity,c1.from);
	//call_t cs = {9, SINK, c->from};
	struct nodedata *nodedata = get_node_private_data(&c1);

	struct neighbor *neighbor = NULL;
	 
     das_init_traverse(nodedata->neighbors); 
     int number = 0;     
     while ((neighbor = (struct neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
	//fprintf(fl,"nbid %d cid %d",neighbor->id,c->node);
        if (neighbor->id == c->node) {
            number = neighbor->hello_number;
	    break;
	}
	//fprintf(fl,"\n");
     }
	//fprintf(fl,"c %d hello_number %d\n",c->node,number);
	double df = number/3;
	FILE *fle;
	fle = fopen("lq_nb.txt","ab");
	fprintf(fle,"nb %d df %f dr %f\n",nb->id,df,dr);
	fclose(fle);
	double lq1 = dr*df;
	double lq2 = 1/lq1;*/
	nb->lq = dr;
	if(nb->lq>1)
	    nb->lq = 1;
	//fprintf(fl,"nb %d lq %f\n",nb->id,nb->lq);
	//fclose(fl);
	return nb->lq;

}


void update_wait(call_t *c, int from, uint64_t queue_delay){
	//printf("%s\n","update_wait_start");
	int id_node = c->node;
	//int from = (c->node)%NUMBER;
	int delay = queue_delay;
        //printf("node %d route_receive queue_delay %d\n", c->node, delay);
	call_t c0 = {c->entity,id_node,6};
        if(c->node == SINK)
            c0.entity = 9;
        //printf("form %d node %d entity %d\n", c0.from, c0.node, c0.entity);
	struct nodedata *nodedata = get_node_private_data(&c0);
	struct neighbor *neighbor = NULL;
	struct neighbor *from_neighbor = NULL;
	das_init_traverse(nodedata->neighbors); 	 
	while ((neighbor = (struct neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
		if (neighbor->id == from) {
			from_neighbor = neighbor;
			break;
		}
	}
	if(from_neighbor==NULL)
		return;
	uint32_t *wait = from_neighbor->wait;
	uint32_t total_delay = 0;
	uint32_t del_delay = 0;
	
	int i;
	for(i=0;i<from_neighbor->wait_size;i++){
		total_delay = total_delay + *(from_neighbor->wait+i);
	}
	if(from_neighbor->window_size==0)
		//from_neighbor->wait_time = B*delay;//update wait_time
		from_neighbor->wait_time = delay;//update wait_time
	else{
		from_neighbor->wait_time = (1-B)*total_delay/from_neighbor->wait_size+B*delay;//update wait_time
	}

	if(from_neighbor->wait_size==N){
		for(i=0;i<from_neighbor->wait_size-1;i++){
			*(from_neighbor->wait+i)=*(from_neighbor->wait+i+1);
		}
		*(from_neighbor->wait+from_neighbor->wait_size-1) = from_neighbor->wait_time;
	}
	else{
		*(from_neighbor->wait+from_neighbor->wait_size) = from_neighbor->wait_time;
		from_neighbor->wait_size++;
	}
	//printf("%s\n","update_wait_end");
	return;
}

void update_Q(call_t *c, int from, uint64_t mac_delay){
	//printf("%s\n","updateq_start");
	int delay = mac_delay;
        //printf("node %d get_mac_delay %d\n", c->node, delay);
	int id_node = c->node;
	//int from = (c->node)%NUMBER;
	call_t c0 = {c->entity,id_node,6};
	struct nodedata *nodedata = get_node_private_data(&c0);
	struct neighbor *neighbor = NULL;
	struct neighbor *from_neighbor = NULL;
	FILE *fp;
	fp = fopen("next.txt","ab");
	//fprintf(fl,"%s\n","updateq_start");
	//fprintf(fl,"ack from %d total %d\n",from,c->node);
	das_init_traverse(nodedata->neighbors); 	 
	while ((neighbor = (struct neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
		//fprintf(fl,"[updateq_start] node %d nb %d from %d total %d\n",id_node,neighbor->id,from,c->node);
		//fprintf(fl,"nb %d\n",neighbor->id);
		if (neighbor->id == from) {
			from_neighbor = neighbor;
			break;
		}
	}
	
	if(from_neighbor==NULL){
		//fprintf(fl,"%s\n","fromnb null");
		//fclose(fl);
		return;
	}
	//fclose(fl);
	if(delay>nodedata->nb_maxdelay)
		nodedata->nb_maxdelay = delay;
	uint32_t *windows = from_neighbor->windows;
	uint32_t total_delay = 0;
	uint32_t del_delay = 0;
	uint32_t avg_delay = 0;
	uint32_t s_delay = 0;
	int i;
	/*FILE *f;
	f = fopen("adelay.txt","ab");*/
	for(i=0;i<from_neighbor->window_size;i++){
		total_delay = total_delay + *(from_neighbor->windows+i);
	//	fprintf(f,"i %d window_delay %u\n",i,*(from_neighbor->windows+i));
	}
	//fclose(f);
	//fprintf(f,"id %d size %d\n",from,from_neighbor->window_size);
	if(from_neighbor->window_size==0){
		//from_neighbor->delay = B*delay;//update delay
		
		//fprintf(f,"nb_id %d delay %u\n",from, from_neighbor->delay);
		from_neighbor->delay = delay;//update delay
	}
	else{
		from_neighbor->delay = (1-B)*total_delay/from_neighbor->window_size+B*delay;//update delay
	}
	from_neighbor->macd = from_neighbor->delay;
	from_neighbor->delay += from_neighbor->wait_time;
	
	
	//fprintf(f,"before node %d nb %d delay %u\n",id_node,from,*(from_neighbor->windows+from_neighbor->window_size-1));
	//fprintf(f,"after node %d nb %d delay %u\n",id_node,from,from_neighbor->delay);
	//fclose(f);
	/* a */
	if(from_neighbor->window_size==N){
		del_delay = *(from_neighbor->windows);
		for(i=0;i<from_neighbor->window_size-1;i++){
			*(from_neighbor->windows+i)=*(from_neighbor->windows+i+1);
		}
		*(from_neighbor->windows+from_neighbor->window_size-1) = from_neighbor->delay;
		total_delay = total_delay+from_neighbor->delay-del_delay;
	}
	else{
		//printf("insert where %d\n",replay_neighbor->window_size);
		*(from_neighbor->windows+from_neighbor->window_size) = from_neighbor->delay;
		from_neighbor->window_size++;
		total_delay = total_delay+from_neighbor->delay;
	}
	
	avg_delay = total_delay/from_neighbor->window_size;
	
	uint32_t delay_size = avg_delay;
	int count = 0;
	while(delay_size!=0){
		delay_size/=10;
		count++;
	}
	
	//FILE *p=fopen("next.txt","ab");
	for(i=0;i<from_neighbor->window_size;i++){
		//fprintf(p,"s_delay %u\n",s_delay);
		//s_delay += pow((*(from_neighbor->windows+i))/(pow(10,count-2)*1.0)-avg_delay/(pow(10,count-2)*1.0),2);
		s_delay += pow(*(from_neighbor->windows+i)-avg_delay*1.0,2);
		//fprintf(p,"delay %u  avg_delay %u s_delay %u\n",*(from_neighbor->windows+i),avg_delay,s_delay);
	}
	
	
	int scount=0;
	
	//fprintf(p,"bf_s_delay %u\n",s_delay);
	s_delay = sqrt(s_delay/from_neighbor->window_size);
	delay_size = s_delay;
	//fprintf(p,"af_s_delay %u\n",s_delay);
	//fprintf(p,"delay_size %u\n",delay_size);
	
	/*while(delay_size!=0){
		delay_size/=10;
		scount++;
	}*/
	//s_delay = s_delay/pow(10,scount-count+1);
	
	//fprintf(p,"from_neighbor->delay  %u  avg_delay %u  s_delay %u\n",from_neighbor->delay,avg_delay,s_delay);
	
	if(s_delay==0)
		from_neighbor->a = 0.3;
	else{
		double t = abs(from_neighbor->delay-avg_delay)/(s_delay*1.0);
		from_neighbor->a =1-exp(0-t);
		//from_neighbor->a = 2*exp(0-t);
	}
		//from_neighbor->a = (abs(from_neighbor->delay-avg_delay)/pow(10,count-2)*1.0)/s_delay;
	if(from_neighbor->a>1)
		from_neighbor->a = 1;

	/* q */
	double b_delay = (double)(from_neighbor->delay)/1.0;
	double e_delay = exp(0-1.0*b_delay/nodedata->nb_maxdelay);
	double s_energy = from_neighbor->energy/(ENERGY*1.0);
	double reward;
	if(from_neighbor->id==SINK){
		reward = 1;
	}
	else
		reward = W*e_delay+(1-W)*s_energy;
	//fprintf(p,"nb_energy %f ",from_neighbor->energy);
	//fprintf(p,"NERGY %f\n",ENERGY);
	//fprintf(p,"e_delay %f s_energy %f\n",e_delay,s_energy);

	
	//if(from_neighbor->q<0)
		//printf("%s\n","CQ");
	
	double old_q = from_neighbor->q;
	from_neighbor->q = (1-from_neighbor->a)*from_neighbor->q+from_neighbor->a*(reward+(from_neighbor->r)*from_neighbor->max_q);
        fprintf(fp, "old_q %f a %f reward %f max_q %f r %f\n", old_q, from_neighbor->a, reward, from_neighbor->max_q, from_neighbor->r);
	double new_q = from_neighbor->q;
        fprintf(fp, "reward %f\n", reward);
	fprintf(fp, "[update_q] node %d nb %d nb_q %f a %f  \n",id_node,from_neighbor->id,from_neighbor->q,from_neighbor->a);
	fprintf(fp,"r %f nb_maxq %f\n",from_neighbor->r,from_neighbor->max_q);
	fclose(fp);
	//printf("%s\n","updateq_end");
	return;

}

/*void update_Q(call_t *c, packet_t *packet,int type,uint64_t backtrack){
	struct nodedata *nodedata = get_node_private_data(c);
	struct data_send * data_se = NULL;
	uint32_t nb_delay=0;
	struct replay_p *replay;
	struct ack_p *ack;

	nodeid_t res_id = -1;
	switch (type){
		case REPLAY_PACKET:
			replay = (struct replay_p *) (packet->data + nodedata->overhead);
			res_id = replay->res;
			nb_delay = backtrack-nodedata->start_hello;
			break;
		case ACK_PACKET:
			ack = (struct ack_p *) (packet->data + nodedata->overhead);
			res_id = ack->res;
			das_init_traverse(nodedata->data_st); 	 
			while ((data_se = (struct data_send *) das_traverse(nodedata->data_st)) != NULL) {
				if (data_se->seq==res_id) {
					nb_delay = backtrack-data_se->send;
					data_se->seq = -1;
					break;
				}
			}

			break;
	}
	
	//printf("nb_delay %u\n",nb_delay);

	//uint64_t *windows = NULL;////////////
	struct neighbor *neighbor = NULL;
	struct neighbor *replay_neighbor = NULL;
	das_init_traverse(nodedata->neighbors); 	 
	while ((neighbor = (struct neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
		if (neighbor->id == res_id) {
			//windows = neighbor->windows;
			replay_neighbor = neighbor;
			//printf("neighbor_id %d res_id %d\n",neighbor->id,res_id);
			break;
		}
	}
	if(replay_neighbor==NULL){
		
		return;
	}
	uint32_t *windows = replay_neighbor->windows;
	uint32_t total_delay = 0;
	uint32_t del_delay = 0;
	uint32_t avg_delay = 0;
	double s_delay = 0;
	int i;
	for(i=0;i<replay_neighbor->window_size;i++){
		total_delay = total_delay + *(replay_neighbor->windows+i);
		//printf("window_delay %u\n",*(replay_neighbor->windows+i));
	}
	if(replay_neighbor->window_size==0)
		replay_neighbor->delay = B*nb_delay;//update delay
	else{
		replay_neighbor->delay = (1-B)*total_delay/replay_neighbor->window_size+B*nb_delay;//update delay
	}
	double error_locatation = locate_error(c,replay_neighbor);
	if(error_locatation<1)
		replay_neighbor->delay = replay_neighbor->delay/(1-error_locatation);
	

	
	if(replay_neighbor->window_size==N){
		del_delay = *(replay_neighbor->windows);
		for(i=0;i<replay_neighbor->window_size-1;i++){
			*(replay_neighbor->windows+i)=*(replay_neighbor->windows+i+1);
		}
		*(replay_neighbor->windows+replay_neighbor->window_size-1) = replay_neighbor->delay;
		total_delay = total_delay+replay_neighbor->delay-del_delay;
	}
	else{
		//printf("insert where %d\n",replay_neighbor->window_size);
		*(replay_neighbor->windows+replay_neighbor->window_size) = replay_neighbor->delay;
		replay_neighbor->window_size++;
		total_delay = total_delay+replay_neighbor->delay;
	}
	//printf("neighbor_id %d neighbor_size %d :",replay_neighbor->id,replay_neighbor->window_size);
	
	
	avg_delay = total_delay/replay_neighbor->window_size;
	
	uint32_t delay_size = avg_delay;
	int count = 0;
	while(delay_size!=0){
		delay_size/=10;
		count++;
	}
	
	for(i=0;i<replay_neighbor->window_size;i++){
		s_delay += pow((*(replay_neighbor->windows+i))/(pow(10,count-2)*1.0)-avg_delay/(pow(10,count-2)*1.0),2);
	}
	s_delay = s_delay/replay_neighbor->window_size;
	if(s_delay==0)
		replay_neighbor->a = 1;
	else
		replay_neighbor->a = (abs(nb_delay-avg_delay)/pow(10,count-2)*1.0)/s_delay;
	
	//FILE *fpt;
	//fpt = fopen("print.txt","ab");
	//printf("neighbor_id %d a %f\n",replay_neighbor->id,replay_neighbor->a);
	double b_delay = (double)(replay_neighbor->delay)/1000000.0;
	double e_delay = exp(0-b_delay);
	double s_energy = replay_neighbor->energy/(ENERGY*1.0);
	double imm_re = W*e_delay+(1-W)*s_energy;

	printf("node %d nb %d nb_q %f a %f r %f nb_maxq %f\n",c->node,replay_neighbor->id,replay_neighbor->q,replay_neighbor->a,replay_neighbor->r,replay_neighbor->max_q);

	replay_neighbor->q = (1-replay_neighbor->a)*replay_neighbor->q+replay_neighbor->a*(imm_re+(replay_neighbor->r)*replay_neighbor->max_q);
	//printf("Node %d neighbor %d neighbor_q %f neighbor_energy %d neighbor_a %f neighbor_r %f imme_return %f neighbor_delay %u \n",c->node,replay_neighbor->id,replay_neighbor->q,replay_neighbor->energy,replay_neighbor->a,replay_neighbor->r,imm_re,replay_neighbor->delay);
	//printf("%s \n","end_updateq");
	//fclose(fpt);
	return;
	
}*/


/* ************************************************** */
/* ************************************************** */
/*struct neighbor* get_nexthop(call_t *c, position_t *dst, packet_t *packet) {//根据速度和Q值获取下一跳
    	struct nodedata *nodedata = get_node_private_data(c);
	void *candidate_neighbors = das_create();
    	struct neighbor *neighbor = NULL, *n_hop = NULL, *max_speed_nb = NULL, *maxq_nb = NULL;
	maxq_nb = (struct neighbor *) malloc(sizeof(struct neighbor));
	maxq_nb->prospeed=0;
	maxq_nb->id = c->node;
	struct data_p *data = (struct data_p *) (packet->data + nodedata->overhead);

	call_t cs = {9, SINK, c->from};
	dst->x = get_node_position(cs.node)->x;
	dst->y = get_node_position(cs.node)->y;
	dst->z = get_node_position(cs.node)->z;
	double dist = distance(get_node_position(c->node), dst);
	
	double req_speed = dist/(data->deadline/1000000000.0);
    
    	uint64_t clock = get_time();
	double d = 0, d_min = 0, d_max = 0, mobility_r = 0;
	double pro_speed = 0, pro_speed_avg = 0, pro_speed_min = 0, pro_speed_max = 0;
	double max_speed = 0;
	double q_value = 0;
	double maxsp_d = 0;
	

    // parse neighbors 
    int size_nb = 0;  
    double sum_q = 0;
    das_init_traverse(nodedata->neighbors); 
    //int size_nb = das_getsize(nodedata->neighbors); 
    printf("node %d get_nexthop nb_number %d packet %d\n", c->node, size_nb, packet->id);
    FILE *fp = fopen("next.txt", "ab");
    fprintf(fp,"node %d get_nexthop nb_number %d packet %d\n", c->node, size_nb, packet->id);  
    while ((neighbor = (struct neighbor *) das_traverse(nodedata->neighbors)) != NULL) { 
		//size_nb++; 
		entity_t *entity1 = get_entity_by_name("liner");
		int c_id = entity1->id;
		call_t c1={c_id,neighbor->id,-1};
		double status = entity1->methods->energy.energy_status(&c1);
		double remain_energy = entity1->methods->energy.energy_remaining(&c1);
		if(status==0||remain_energy<0)
			continue;
        if ((nodedata->nb_timeout > 0)
            && (clock - neighbor->time) >= nodedata->nb_timeout ) {
            continue;
        }
        
        // choose candidate_neighbor 
		//mobility_r = (clock-neighbor->time)*neighbor->speed;
		position_t pos;
		pos.x = neighbor->position.x+((neighbor->speed * cos(neighbor->angle.xy)*cos(neighbor->angle.z))*(clock-neighbor->time+neighbor->macd))/1000000000;
		pos.y = neighbor->position.y+((neighbor->speed * sin(neighbor->angle.xy)*cos(neighbor->angle.z))*(clock-neighbor->time+neighbor->macd))/1000000000;
		pos.z = neighbor->position.z+((neighbor->speed * sin(neighbor->angle.z))*(clock-neighbor->time+neighbor->macd))/1000000000;
		if(pos.z<0)
			pos.z = 0;
		if(pos.x>SINK_X)
			pos.x=SINK_X;
		if(pos.y>SINK_Y)
			pos.y=SINK_Y;
		double d_new = distance(dst,&(pos));
		
       // d = distance(&(neighbor->position), dst);
		//fprintf(fp,"Node %d neighbor %d neighbor_delay %u\n",c->node,neighbor->id,neighbor->delay);
		if(neighbor->delay==0)
			continue;
		double rela_rate = neighbor_relation(c,neighbor);
		//fprintf(fp,"rela_rate %f\n",rela_rate);
		if(rela_rate==0)
			continue;
		if(neighbor->id==data->src)
			continue;
		double lq = get_neighbor_lq(neighbor,c);
		//fprintf(fp,"lq1 %f lq2 %f\n",lq,neighbor->lq);
		double tq = rela_rate*lq;
		//double tq = 1;
		if(neighbor->delay!=0){
			pro_speed = (dist-d_new)/(neighbor->delay/1000000000.0);
			neighbor->prospeed = pro_speed;
			//pro_speed = (dist-d)/(neighbor->delay/1000000000.0);
		}
		//double error_locate = locate_error(c,neighbor);
		//fprintf(fp,"nb_a %f nb_r %f\n",neighbor->a,neighbor->r);
		fprintf(fp,"Node %d neighbor %d nb_q %f max_q %f req_speed %f pro_speed %f \n",c->node,neighbor->id,neighbor->q,neighbor->max_q,req_speed,pro_speed);
		//fprintf(fp,"delay %u\n",neighbor->delay);
		
		if(neighbor->prospeed>=req_speed){
			if(neighbor->id==data->dst){
				fclose(fp);
				return neighbor;
			}
			das_insert(candidate_neighbors, (void *) neighbor);
			sum_q += neighbor->q;
			size_nb++;
		}
		if(pro_speed>max_speed){
			max_speed = pro_speed;
			max_speed_nb = neighbor;
			//maxsp_d = d;
		}
    }
	//fprintf(fp,"%s %d\n","nb_number",size_nb);
	//FILE *fcq;
	//fcq = fopen("getneighbor_q.txt","ab"); 
	int size = das_getsize(candidate_neighbors);
	fprintf(fp, "candidate_neighbors_size %d size_nb %d \n",size,size_nb);
        printf("candidate_neighbors_size %d size_nb %d \n",size,size_nb);
	//fclose(fp);
	//FILE *f;
	//f = fopen("next.txt","ab");
	if(size==0){
		fprintf(fp,"1maxspeed %f reqspeed %f \n",max_speed,req_speed);
		printf("1maxspeed %f reqspeed %f \n",max_speed,req_speed);
		if(max_speed==0){
			fclose(fp);
			//fclose(fcq);
			printf("%s \n","end_nexthop");	
			return NULL;
		}
		else{
			n_hop = max_speed_nb;
			fprintf(fp,"2maxspeed %f reqspeed %f \n",max_speed,req_speed);
			//fprintf(fcq,"2maxspeed %f reqspeed %f \n",max_speed,req_speed);
			//uint64_t delay_lowsp = (uint64_t)(((dist-maxsp_d)/max_speed-(dist-maxsp_d)/req_speed)*1000000000);
			//data->deadline -= delay_lowsp;
		}
	}
	else{
		struct neighbor *c_neighbor = NULL;
		//double r = rand() % (999 + 1) / (double)(999 + 1);; //r为0至1的随机数
		//double m = 0;
                double max_q = 0;
                printf("traves candidate neighbors\n");
		das_init_traverse(candidate_neighbors); 
    		while ((c_neighbor = (struct neighbor *) das_traverse(candidate_neighbors)) != NULL) {
                        //printf("candnb %d\n", c_neighbor->id);
			double rela_rate = neighbor_relation(c,c_neighbor);
                        //printf("11 \n");
			double lq = get_neighbor_lq(c_neighbor,c);
		        fprintf(fp,"nb %d lq %f rela_rate %f q_value %f\n", c_neighbor->id, lq, rela_rate, c_neighbor->q);
                        //printf("lq %f rela_rate %f q_value %f\n", lq, rela_rate, c_neighbor->q);
			double weightedQ = rela_rate * lq * c_neighbor->q; 
                        fprintf(fp,"weightedQ %f max_q %f\n", weightedQ, max_q);
                        if(weightedQ > max_q){
                              n_hop = c_neighbor;
                              max_q = weightedQ;
                        }
			
		}

	}
	//fclose(fcq);
	if(size!=0&&maxq_nb->id==c->node){
		n_hop = max_speed_nb;
                fprintf(fp, "n_hop = max_speed_nb\n");
	}
	fprintf(fp,"next_nb %d\n",n_hop->id);
	//fprintf(fp,"%s \n","end_nexthop");
    	fclose(fp);
	//printf("%s \n","end_nexthop");
    return n_hop;
}*/

//e greedy
struct neighbor* get_nexthop(call_t *c, position_t *dst,packet_t *packet) {//根据速度和Q值获取下一跳
	//printf("%s \n","start_nexthop");
	FILE *fp;
	fp = fopen("next.txt","ab");
	fprintf(fp,"node %d get_nexthop packet %d\n", c->node, packet->id);
    	struct nodedata *nodedata = get_node_private_data(c);
	void *candidate_neighbors = das_create();
    	struct neighbor *neighbor = NULL, *n_hop = NULL, *maxsp_nb = NULL, *maxq_nb = NULL;
	maxq_nb = (struct neighbor *) malloc(sizeof(struct neighbor));
	maxq_nb->prospeed=0;
	maxq_nb->id = c->node;
	struct data_p *data = (struct data_p *) (packet->data + nodedata->overhead);

	call_t cs = {9, SINK, c->from};
	dst->x = get_node_position(cs.node)->x;
	dst->y = get_node_position(cs.node)->y;
	dst->z = get_node_position(cs.node)->z;
	double dist = distance(get_node_position(c->node), dst);
	
	double req_speed = dist/(data->deadline/1000000000.0);
    
    	uint64_t clock = get_time();
	double d = 0, d_min = 0, d_max = 0, mobility_r = 0;
	double pro_speed = 0, pro_speed_avg = 0, pro_speed_min = 0, pro_speed_max = 0;
	double max_speed = 0;
	double q_value = 0;
	double maxsp_d = 0;
	

    // parse neighbors 
    int size_nb = 0;  
    das_init_traverse(nodedata->neighbors); 
    //int size_nb = das_getsize(nodedata->neighbors); 
    //fprintf(fp,"%s %n\n","nb_number",size_nb); 
    int k =1; 
    while ((neighbor = (struct neighbor *) das_traverse(nodedata->neighbors)) != NULL) { 
		size_nb++; 
		entity_t *entity1 = get_entity_by_name("liner");
		int c_id = entity1->id;
		call_t c1={c_id,neighbor->id,-1};
		double status = entity1->methods->energy.energy_status(&c1);
		double remain_energy = entity1->methods->energy.energy_remaining(&c1);
		if(status==0||remain_energy<0)
			continue;
        if ((nodedata->nb_timeout > 0)
            && (clock - neighbor->time) >= nodedata->nb_timeout ) {
            continue;
        }
        
        // choose candidate_neighbor
		//mobility_r = (clock-neighbor->time)*neighbor->speed;
		position_t pos;
		pos.x = neighbor->position.x+((neighbor->speed * cos(neighbor->angle.xy)*cos(neighbor->angle.z))*(clock-neighbor->time+neighbor->macd))/1000000000;
		pos.y = neighbor->position.y+((neighbor->speed * sin(neighbor->angle.xy)*cos(neighbor->angle.z))*(clock-neighbor->time+neighbor->macd))/1000000000;
		pos.z = neighbor->position.z+((neighbor->speed * sin(neighbor->angle.z))*(clock-neighbor->time+neighbor->macd))/1000000000;
		if(pos.z<0)
			pos.z = 0;
		if(pos.x>SINK_X)
			pos.x=SINK_X;
		if(pos.y>SINK_Y)
			pos.y=SINK_Y;
		double d_new = distance(dst,&(pos));
		
		//fprintf(fp,"Node %d neighbor %d neighbor_delay %u\n",c->node,neighbor->id,neighbor->delay);
		if(neighbor->delay==0)
			continue;
		double rela_rate = neighbor_relation(c,neighbor);
		//fprintf(fp,"rela_rate %f\n",rela_rate);
		if(rela_rate==0)
			continue;
		if(neighbor->id==data->src)
			continue;
		double lq = get_neighbor_lq(neighbor,c);
		//fprintf(fp,"lq1 %f lq2 %f\n",lq,neighbor->lq);
		double tq = rela_rate*lq;
		//double tq = 1;
		if(neighbor->delay!=0){
			pro_speed = (dist-d_new)/(neighbor->delay/1000000000.0);
			neighbor->prospeed = pro_speed;
			//pro_speed = (dist-d)/(neighbor->delay/1000000000.0);
		}
		//double error_locate = locate_error(c,neighbor);
		//fprintf(fp,"nb_a %f nb_r %f\n",neighbor->a,neighbor->r);
		fprintf(fp,"Node %d neighbor %d req_speed %f pro_speed %f\n",c->node,neighbor->id,req_speed,pro_speed);
                fprintf(fp,"tq %f nb_q %f max_q %f\n", tq, neighbor->q*tq, q_value);
		//fprintf(fp,"delay %u\n",neighbor->delay);
		
		if(neighbor->prospeed>=req_speed){
			if(neighbor->id==data->dst){
				fclose(fp);
				return neighbor;
			}
			das_insert(candidate_neighbors, (void *) neighbor);
			neighbor->cid = k;
			k++;
			
			if(neighbor->q*tq==q_value){
				if(neighbor->prospeed>maxq_nb->prospeed){
					//q_value = neighbor->q*rela_rate;
					maxq_nb = neighbor;
				}
			}
			if(neighbor->q*tq>q_value){//////////////////
				q_value = neighbor->q*tq;
				maxq_nb = neighbor;
			}
		}
		if(pro_speed>max_speed){
			max_speed = pro_speed;
			maxsp_nb = neighbor;
			maxsp_d = d;
		}
    }
	//fprintf(fp,"%s %d\n","nb_number",size_nb); 
	int size = das_getsize(candidate_neighbors);
	fprintf(fp,"candidate_neighbors_size %d\n",size);
	//fclose(fp);
	//FILE *f;
	//f = fopen("next.txt","ab");
	if(size==0){
		fprintf(fp,"1maxspeed %f reqspeed %f \n",max_speed,req_speed);
		if(max_speed==0){
			fclose(fp);
			//printf("%s \n","end_nexthop");	
			return NULL;
		}
		else{
			n_hop = maxsp_nb;
			fprintf(fp,"2maxspeed %f reqspeed %f \n",max_speed,req_speed);
			//uint64_t delay_lowsp = (uint64_t)(((dist-maxsp_d)/max_speed-(dist-maxsp_d)/req_speed)*1000000000);
			//data->deadline -= delay_lowsp;
		}
	}
	else{
		n_hop = maxq_nb;
		fprintf(fp,"%s \n","size!=0");
	}
	if(size!=0&&maxq_nb->id==c->node){
		n_hop = maxsp_nb;
	}
	fprintf(fp,"next_nb %d\n",n_hop->id);
	fprintf(fp,"%s \n","end_nexthop");
    	fclose(fp);
	printf("%s \n","end_nexthop");
    return n_hop;
}


//UCB
/*struct neighbor* get_nexthop(call_t *c, position_t *dst,packet_t *packet) {//根据速度和Q值获取下一跳
	printf("%s \n","start_nexthop");
	FILE *fp;
	fp = fopen("next.txt","ab");
	fprintf(fp,"%s \n","start_nexthop");
    	struct nodedata *nodedata = get_node_private_data(c);
	void *candidate_neighbors = das_create();
    	struct neighbor *neighbor = NULL, *n_hop = NULL, *maxsp_nb = NULL, *maxq_nb = NULL;
	maxq_nb = (struct neighbor *) malloc(sizeof(struct neighbor));
	maxq_nb->prospeed=0;
	maxq_nb->id = c->node;
	struct data_p *data = (struct data_p *) (packet->data + nodedata->overhead);

	call_t cs = {9, SINK, c->from};
	dst->x = get_node_position(cs.node)->x;
	dst->y = get_node_position(cs.node)->y;
	dst->z = get_node_position(cs.node)->z;
	double dist = distance(get_node_position(c->node), dst);
	
	double req_speed = dist/(data->deadline/1000000000.0);
    
    	uint64_t clock = get_time();
	double d = 0, d_min = 0, d_max = 0, mobility_r = 0;
	double pro_speed = 0, pro_speed_avg = 0, pro_speed_min = 0, pro_speed_max = 0;
	double max_speed = 0;
	double q_value = 0;
	double maxsp_d = 0;
	

    // parse neighbors 
    int size_nb = 0;  
    das_init_traverse(nodedata->neighbors); 
    //int size_nb = das_getsize(nodedata->neighbors); 
    //fprintf(fp,"%s %n\n","nb_number",size_nb);  
    while ((neighbor = (struct neighbor *) das_traverse(nodedata->neighbors)) != NULL) { 
		size_nb++; 
		entity_t *entity1 = get_entity_by_name("liner");
		int c_id = entity1->id;
		call_t c1={c_id,neighbor->id,-1};
		double status = entity1->methods->energy.energy_status(&c1);
		double remain_energy = entity1->methods->energy.energy_remaining(&c1);
		if(status==0||remain_energy<0)
			continue;
        if ((nodedata->nb_timeout > 0)
            && (clock - neighbor->time) >= nodedata->nb_timeout ) {
            continue;
        }
        
        // choose candidate_neighbor
		//mobility_r = (clock-neighbor->time)*neighbor->speed;
		position_t pos;
		pos.x = neighbor->position.x+((neighbor->speed * cos(neighbor->angle.xy)*cos(neighbor->angle.z))*(clock-neighbor->time+neighbor->macd))/1000000000;
		pos.y = neighbor->position.y+((neighbor->speed * sin(neighbor->angle.xy)*cos(neighbor->angle.z))*(clock-neighbor->time+neighbor->macd))/1000000000;
		pos.z = neighbor->position.z+((neighbor->speed * sin(neighbor->angle.z))*(clock-neighbor->time+neighbor->macd))/1000000000;
		if(pos.z<0)
			pos.z = 0;
		if(pos.x>SINK_X)
			pos.x=SINK_X;
		if(pos.y>SINK_Y)
			pos.y=SINK_Y;
		double d_new = distance(dst,&(pos));
		
		fprintf(fp,"Node %d neighbor %d neighbor_delay %u\n",c->node,neighbor->id,neighbor->delay);
		if(neighbor->delay==0)
			continue;
		double rela_rate = neighbor_relation(c,neighbor);
		fprintf(fp,"rela_rate %f\n",rela_rate);
		if(rela_rate==0)
			continue;
		if(neighbor->id==data->src)
			continue;
		double lq = get_neighbor_lq(neighbor,c);
		fprintf(fp,"lq1 %f lq2 %f\n",lq,neighbor->lq);
		double tq = rela_rate*lq;
		//double tq = 1;
		if(neighbor->delay!=0){
			pro_speed = (dist-d_new)/(neighbor->delay/1000000000.0);
			neighbor->prospeed = pro_speed;
			//pro_speed = (dist-d)/(neighbor->delay/1000000000.0);
		}
		//double error_locate = locate_error(c,neighbor);
		//fprintf(fp,"nb_a %f nb_r %f\n",neighbor->a,neighbor->r);
		fprintf(fp,"Node %d neighbor %d nb_q %f max_q %f req_speed %f pro_speed %f dist %f   ",c->node,neighbor->id,neighbor->q,neighbor->max_q,req_speed,pro_speed,dist);
		fprintf(fp,"delay %u\n",neighbor->delay);
		
		if(neighbor->prospeed>=req_speed){
			if(neighbor->id==data->dst){
				fclose(fp);
				return neighbor;
			}
			das_insert(candidate_neighbors, (void *) neighbor);

			double qn;
			if(neighbor->choose_num>0)
				qn = neighbor->q+sqrt(1/neighbor->choose_num);
			else
				qn = neighbor->q;
			
			if(qn>q_value){//////////////////
				q_value = qn;
				maxq_nb = neighbor;
			}
		}
		if(pro_speed>max_speed){
			max_speed = pro_speed;
			maxsp_nb = neighbor;
			maxsp_d = d;
		}
    }
	fprintf(fp,"%s %d\n","nb_number",size_nb); 
	int size = das_getsize(candidate_neighbors);
	fprintf(fp,"candidate_neighbors_size %d\n",size);
	//fclose(fp);
	//FILE *f;
	//f = fopen("next.txt","ab");
	if(size==0){
		fprintf(fp,"1maxspeed %f reqspeed %f \n",max_speed,req_speed);
		if(max_speed==0){
			fclose(fp);
			printf("%s \n","end_nexthop");	
			return NULL;
		}
		else{
			n_hop = maxsp_nb;
			fprintf(fp,"2maxspeed %f reqspeed %f \n",max_speed,req_speed);
			//uint64_t delay_lowsp = (uint64_t)(((dist-maxsp_d)/max_speed-(dist-maxsp_d)/req_speed)*1000000000);
			//data->deadline -= delay_lowsp;
		}
	}
	else{
		fprintf(fp,"%s \n","size!=0");
		n_hop = maxq_nb;
	}
	if(size!=0&&maxq_nb->id==c->node){
		n_hop = maxsp_nb;
	}
	//fprintf(fp,"next_nb %d\n",n_hop->id);
	fprintf(fp,"%s \n","end_nexthop");
    	fclose(fp);
	printf("%s \n","end_nexthop");
    return n_hop;
}*/



void init_q(call_t *c,struct neighbor *nb){
	uint64_t T;
	T = (DATA_SIZE+100)*T_b;
	
	int delay = T;
	int id_node = c->node;
	int from = nb->id;
	
	struct nodedata *nodedata = get_node_private_data(c);
	struct neighbor *neighbor = nb;
	
	
	uint32_t *windows = neighbor->windows;
	uint32_t total_delay = 0;
	uint32_t del_delay = 0;
	uint32_t avg_delay = 0;
	double s_delay = 0;
	int i;
	
	neighbor->delay = T;
	neighbor->macd = T;

	if(neighbor->delay>nodedata->nb_maxdelay)
		nodedata->nb_maxdelay = neighbor->delay;
	*(neighbor->windows+neighbor->window_size) = neighbor->delay;
	
	neighbor->q = 0.5;
	return;
}

void add_neighbor(call_t *c, struct hello_p *hello) {
	 struct nodedata *nodedata = get_node_private_data(c);
	 struct neighbor *neighbor = NULL;
	 
     das_init_traverse(nodedata->neighbors);      
     while ((neighbor = (struct neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
        if (neighbor->id == hello->src) {
            neighbor->position.x = hello->src_pos.x;
            neighbor->position.y = hello->src_pos.y;
            neighbor->position.z = hello->src_pos.z;
            neighbor->time       = get_time();
	    neighbor->energy = hello->energy;
	    neighbor->r = hello->r;
	    neighbor->max_q = hello->max_q;
	    neighbor->speed = hello->speed;
	    neighbor->angle = hello->angle;
	    neighbor->choose_num = 0;
	    neighbor->ilde = hello->ilde;
	    
	    neighbor->cid = 0;
		if(get_time()-nodedata->hello_time<=3*nodedata->hello_period)
	    		neighbor->hello_number++;
		else
			neighbor->hello_number = 1;
        return;
        }
    }  

    /* new neighbor */
	//printf("[add_nb] Node %d neighbor %d node_nboldsize %d\n",c->node,neighbor->id,das_getsize(nodedata->neighbors));
	nodedata->add_nb++;
	
    	neighbor = (struct neighbor *) malloc(sizeof(struct neighbor));
    	neighbor->id         = hello->src;
    	neighbor->position.x = hello->src_pos.x;
    	neighbor->position.y = hello->src_pos.y;
    	neighbor->position.z = hello->src_pos.z;
    	neighbor->time       = get_time();
	neighbor->a = 1;
	neighbor->link       = get_random_double_range(MIN_LINK, MAX_LINK);
	//neighbor->error_location = locate_error(c,neighbor);
	neighbor->q = 0;
	neighbor->prospeed = 0;
	neighbor->max_q = hello->max_q;
	neighbor->r = hello->r;
	neighbor->delay = 0;
	neighbor->wait_time = 0;
	neighbor->windows = (uint32_t *)malloc(sizeof(uint32_t)*N);
	neighbor->window_size = 0;
	neighbor->wait = (uint32_t *)malloc(sizeof(uint32_t)*N);
	neighbor->wait_size = 0;
	neighbor->energy = hello->energy;
	neighbor->lq = rand()/((RAND_MAX+1.0) * 5);
        //printf("add nb lq %f \n", neighbor->lq);
	neighbor->speed = hello->speed;
	neighbor->angle = hello->angle;
        neighbor->hello_number = 1;
	neighbor->ilde = hello->ilde;

	neighbor->cid = 0;
	neighbor->choose_num = 0;
	int i;
	
	for(i=0;i<N;i++){
		*(neighbor->windows+i) = 0;
		*(neighbor->wait+i) = 0;
	}
	neighbor->energy = hello->energy;
	
	init_q(c,neighbor);
    	das_insert(nodedata->neighbors, (void *) neighbor);
    	return;
}


/* ************************************************** */
/* ************************************************** */
int neighbor_timeout(void *data, void *arg) {
    	struct neighbor *neighbor = (struct neighbor *) data;
    	call_t *c = (call_t *) arg;
    	struct nodedata *nodedata = get_node_private_data(c);
    	if ((get_time() - neighbor->time) >= NB_timeout) {
		nodedata->del_nb++;
        	return 1;
    	}
    	return 0;
}

/* ************************************************** */
/* ************************************************** */
void forward(call_t *c, packet_t *packet, uint64_t come_time) {
    	struct nodedata *nodedata = get_node_private_data(c);
	struct data_p *data = (struct data_p *) (packet->data + nodedata->overhead);
    	call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
    	destination_t destination;
	nodeid_t from_id = data->src;
	struct neighbor *neighbor = NULL;
	
	struct timeval start;
	gettimeofday(&start,0);
	struct neighbor *next_nb = get_nexthop(c, &(data->dst_pos), packet);
        printf("node %d forwards packet %d\n", c->node, packet->id);
	struct timeval end;
	gettimeofday(&end,0);
	uint64_t rout = (end.tv_sec-start.tv_sec)*1000000+(end.tv_usec-start.tv_usec);
	data->routt += rout;

        double random = rand()/(RAND_MAX+1.0);
        int nb_num = das_getsize(nodedata->neighbors);
        //printf("random %f\n", random);
        FILE *fp = fopen("next.txt", "ab");
    
        if(random < epl&&(nb_num!=0) && packet->learning_flag == 0){
            //srand(time(NULL));
            //fprintf(fp, "%s\n", "randomly select action");
            int bl = 0;
            while(bl==0){
                double r = rand()%nb_num;
                //printf("r %f nb_num %d\n", r, nb_num);
                int index = 0;
                das_init_traverse(nodedata->neighbors);      
                while ((neighbor = (struct neighbor *) das_traverse(nodedata->neighbors)) != NULL) {
                    if(index == r){
                        if(neighbor->id!=data->src){
                            printf("nb_id %d src %d\n", neighbor->id, data->src);
                            next_nb = neighbor;
                            bl = 1;
                         }
                        if(neighbor->id==data->src && nb_num==1){
                           bl = 1;
                           fprintf(fp, "%s\n","n_hop_null");
                           //fclose(fp);
                           break;
                        }
                        break;
                    }
                    index ++;
                }
            }
        
          }
	

	//printf("%s \n","finish random");
	if(next_nb==NULL){//penalty 
                fprintf(fp, "next_nb null\n");
		struct neighbor *neighbor = NULL;
		nodedata->data_noroute++;
		fprintf(fp,"%s \n","penalty");
		fprintf(fp,"packet_id %d node %d %s \n",packet->id,c->node,"no near next_hop");
		call_t c1 = {c->entity, from_id, c->from};
		// c1;
		if(from_id==0){
			c1.entity = 7;
		}
		else if(from_id==SINK){
			c1.entity = 9;
		}
		else
			c1.entity = 8;
		/*if(from_id==0){
			c1.entity = 7;
		}*/
		/*else{
			c1.entity = 8;
		}*/
		struct nodedata *from_node = get_node_private_data(&c1);
		//printf("start \n");
		//printf("from id %d\n",c1.node);
		//if(from_node==NULL)
		//printf("c->entity %d c->from %d\n",c->entity,c->from);
		int size = das_getsize(from_node->neighbors);
		//if(size==0){
		//printf("size %d\n",size);
		//}
		das_init_traverse(from_node->neighbors);    
    		while ((neighbor = (struct neighbor *) das_traverse(from_node->neighbors)) != NULL) {
			if(neighbor->id==c->node){
				fprintf(fp,"old nb_q %f nb_a %f nb_r %f\n",neighbor->q,neighbor->a,neighbor->r);
				neighbor->q = (1-neighbor->a)*neighbor->q+neighbor->a*(-1+(neighbor->r)*neighbor->max_q);
				//neighbor->q = -1;
				fprintf(fp,"new nb_q %f \n",neighbor->q);
				fprintf(fp,"node %d nb %d %s\n",c->node,neighbor->id,"nb is no neighbor");
				break;
			}
    		}
		
		//printf("end \n");
		/*struct neighbor *t_nb = (struct neighbor *) malloc(sizeof(struct neighbor));
		t_nb->id = c->node;
		t_nb->delay = 0;
		next_nb = t_nb;*/
		fclose(fp);
                return;
			
	}
	
	nodedata->tx_pack++;
	next_nb->choose_num++;
	fprintf(fp, "next_node %d choose_num %d\n", next_nb->id, next_nb->choose_num);
        fclose(fp);

	/* set mac header */
	destination.id = next_nb->id;
	destination.position.x = -1;
	destination.position.y = -1;
	destination.position.z = -1;
	if (SET_HEADER(&c0, packet, &destination) == -1) {/////
		packet_dealloc(packet);
		return;
	}
	
	/*FILE *f;
	f = fopen("hop_delay.txt","ab");
	fprintf(f,"%d %u \n",next_nb->id,next_nb->delay);
	fclose(f);*/
	data->deadline -= next_nb->delay;
	data->src = c->node;
	data->src_pos.x = get_node_position(c->node)->x;
	data->src_pos.y = get_node_position(c->node)->y;
	data->src_pos.z = get_node_position(c->node)->z;
        //printf("node %d transmits pk_id %d\n",c->node,packet->id);
	TX(&c0, packet);

	//struct data_send *datast = (struct data_send *)malloc(sizeof(struct data_send *));
	struct data_send *datast = (struct data_send *)malloc(sizeof(struct data_send));
	//datast->packet = (struct packet *)malloc(sizeof(struct packet));
	datast->send = get_time();
	datast->seq = next_nb->id;
	//datast->packet = packet;
	das_insert(nodedata->data_st, datast);
	//data->deadline = data->deadline-(get_time()-come_time)-packet->size*T_b;
	
		
	//printf("node_id = %d , pack_id = %d , leave_deadline = %u \n",c->node,packet->id,data->deadline);
		
	/* forwarding packet */
	

	entity_t *entity_e = get_entity_by_name("liner");
	int ce_id = entity_e->id;
	call_t ce={ce_id,c->node,-1};
	uint64_t duration = T_b*packet->size;
	entity_e->methods->energy.consume_tx(&ce,duration,0);
	nodedata->energy_node = entity_e->methods->energy.energy_remaining(&ce);
    
	//printf("%s \n","end_forward");
	struct data_send * data_se = NULL;
	uint64_t current = get_time();
	das_init_traverse(nodedata->data_st); 	 
	while ((data_se = (struct data_send *) das_traverse(nodedata->data_st)) != NULL) {
		das_init_traverse(nodedata->neighbors);
		while(((neighbor = (struct neighbor *) das_traverse(nodedata->neighbors)) != NULL)){
			if(neighbor->id==data_se->seq){
				if((current-data_se->send)>=NOT_ACK*neighbor->delay){
					
					neighbor->q = (1-neighbor->a)*neighbor->q+neighbor->a*(-1+(neighbor->r)*neighbor->max_q);
					
					das_delete(nodedata->data_st, data_se);
				}
				break;
			}
		}
	}
	return;
}


int statictics_packet(call_t *c, packet_t *packet){
        //printf("des receives a packet");
	struct nodedata *nodedata = get_node_private_data(c);
	int i;
        //int sum = count -1 ;
	for (i=0;i<count;i++){ //nodedata->rx_pack
		if(*(nodedata->packet_arriveid+i)==packet->id)
			return 0;
	}
	*(nodedata->packet_arriveid+i)=packet->id;
	nodedata->rx_pack++;
        if(packet->learning_flag == 1)
            nodedata->after_learning_rx ++;
	return 1;
}

/* ************************************************** */
/* ************************************************** */
void rx(call_t *c, packet_t *packet) {
	
	struct nodedata *nodedata = get_node_private_data(c);
	
	entity_t *entity_e = get_entity_by_name("liner");
	int ce_id = entity_e->id;
	call_t ce={ce_id,c->node,-1};
	uint64_t duration = T_b*packet->size;
	entity_e->methods->energy.consume_rx(&ce,duration);
	nodedata->energy_node = entity_e->methods->energy.energy_remaining(&ce);
	//FILE *fpt;
	//fpt = fopen("print.txt","ab");
	//printf("%s \n","start_rx");
   
    struct data_d_header *header = (struct data_d_header *) (packet->data + nodedata->overhead);////
    
    switch (header->type) {       

    case HELLO_PACKET:
        /* for us */
       rx_hello(c, packet);
       break;       

    case DATA_PACKET:
        /* for us */
       rx_data(c, packet);
       break;         
   
    default:
       /* not for us */
       packet_dealloc(packet);
       break;
   }
   //printf("%s \n","end_rx");
	//fclose(fpt);
   
    return;
}

/* ************************************************** */
/* ************************************************** */
void rx_hello(call_t *c, packet_t *packet) {//update infromation
    	struct nodedata *nodedata = get_node_private_data(c);
    	struct hello_p *hello = (struct hello_p *) (packet->data + nodedata->overhead); 

	int dist_id = hello->src;
	call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
	position_t *position = get_node_position(c->node);
	add_neighbor(c, hello);
	
   
    return;
}


/* ************************************************** */
/* ************************************************** */
void rx_data(call_t *c, packet_t *packet){
	uint64_t come_time = get_time();
	struct nodedata *nodedata = get_node_private_data(c);
    	struct data_p *data = (struct data_p *) (packet->data + nodedata->overhead);
	call_t c0 = {get_entity_bindings_down(c)->elts[0], c->node, c->entity};
	
	time_t rxtime = time(NULL);
	struct timeval stop;
	gettimeofday(&stop,0);
	//uint64_t del = come_time-data->start_t;
	nodedata->rrxp++;
        printf("node %d receives a packet %d\n", c->node, packet->id);

	/* the node is sink */
	if(c->node==data->dst){
		int bl = statictics_packet(c, packet);
		if(bl==1){
			count++;
                        double delay = (get_time()-data->start_t)/1000000;
			//double delay = (stop.tv_sec-data->start_time.tv_sec)*1000+(stop.tv_usec-data->start_time.tv_usec)/1000;
			if(delay==0){
				return;
			}
			else{
				/*FILE *fl;
				fl = fopen("delay_20.txt","ab");
				fprintf(fl,"%f\n",delay);
				fclose(fl);*/
				nodedata->avg_delay += delay;
				//nodedata->avg_d += del;
				nodedata->avg_hop += data->hop;
                                if(packet->learning_flag == 0){
                                    nodedata->learning_hop += data->hop;
                                }
                                else{
                                    nodedata->test_hop += data->hop;
                                }
				if(delay>nodedata->max_delay){
					nodedata->max_delay = delay;
					nodedata->max_hop = data->hop;
				}		
		
				if(delay<=data->start_deadline)
					nodedata->meet_time++;
			}
                	
		}
		
		packet_dealloc(packet);
		//fclose(fpt);
		return;
	}
	nodedata->rx_pack++;	
	data->hop++;
	nodedata->hop++;
	forward(c,packet,come_time);
	
	//fprintf(fpt,"%s \n","end_rxdata");
	//fclose(fpt);
	//printf("%s \n","end_rx");
	return;
	
}

/*void tx(call_t * c,packet_t *packet){
        FILE *fp = ("next.txt", "ab");
        fprintf(fp, "c_node %d c_form %d packet_id %d\n", c->node, c->from, packet->id);
        fclose(fp);
        printf("node %d from_tx\n", c->node);
	int from = (c->node)%NUMBER;
	int id_node = (c->node)/NUMBER;
	if(c->from<1000000&&c->from>=0){
		update_wait(c);
		return;
	}
	if(c->from<0)
		return;
	update_Q(c);
	call_t c0 = {c->entity,id_node,6};
	struct nodedata *nodedata = get_node_private_data(&c0);
	struct data_send *data_se = NULL;
	int bl;
	das_init_traverse(nodedata->data_st); 	 
	while ((data_se = (struct data_send *) das_traverse(nodedata->data_st)) != NULL) {
		if(data_se->seq==from){
			das_delete(nodedata->data_st, data_se);
			break;
		}
	}
	return;
}*/

void get_queueDelay(call_t *c, int from, packet_t *packet){
    update_wait(c, from, packet->queue_delay);
    printf("get_queueDelay\n");
    return;
}

void get_macDelay(call_t *c, int from, uint64_t mac_delay){
    printf("get_macDelay\n");
    update_Q(c, from, mac_delay);
    return;
}

/* ************************************************** */
/* ************************************************** */
routing_methods_t methods = {rx, get_queueDelay, get_macDelay};

